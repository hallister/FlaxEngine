// Copyright (c) Wojciech Figat. All rights reserved.

#include "SlopeWarp.h"
#include "Engine/Core/Math/Math.h"
#include "Engine/Core/Math/Quaternion.h"
#include "Engine/Graphics/Models/SkeletonData.h"

namespace
{
    // The ground's tilt (its normal's horizontal part against up) below which the ground is taken as flat
    constexpr float FlatSine = 1e-4f;

    bool IsFlat(const Float3& normal)
    {
        return normal.X * normal.X + normal.Z * normal.Z < FlatSine * FlatSine || normal.Y <= ZeroTolerance;
    }
}

Float3 SlopeWarp::GetGroundNormal(const Float3& normal, float alpha, float maxSlopeAngle)
{
    alpha = Math::Saturate(alpha);
    const float length = normal.Length();
    if (length < ZeroTolerance || alpha <= 0.0f || normal.Y <= 0.0f)
        return Float3::UnitY;
    Float3 result = Float3::Lerp(Float3::UnitY, normal / length, alpha);
    result.Normalize();
    const float maxAngle = Math::Clamp(maxSlopeAngle, 0.0f, 89.0f) * DegreesToRadians;
    if (result.Y < Math::Cos(maxAngle))
    {
        Float3 across(result.X, 0, result.Z);
        across.Normalize();
        result = across * Math::Sin(maxAngle) + Float3::UnitY * Math::Cos(maxAngle);
    }
    return result;
}

Real SlopeWarp::GetSlopeHeight(const Vector3& position, const Float3& normal)
{
    if (IsFlat(normal))
        return 0;
    return -((Real)normal.X * position.X + (Real)normal.Z * position.Z) / (Real)normal.Y;
}

bool SlopeWarp::CanWarp(const SkeletonNode* nodes, int32 nodesCount, const Settings& settings)
{
    LegWarp::Chain legs[2];
    return LegWarp::IsValid(settings.Pelvis, nodesCount) && LegWarp::GetLegs(nodes, nodesCount, settings.Pelvis, settings.Legs, legs) != 0;
}

float SlopeWarp::GetPelvisDrop(const SkeletonNode* nodes, int32 nodesCount, const Settings& settings, const Float3& normal, const Transform* pose)
{
    if (settings.MaxPelvisDrop <= 0.0f || !LegWarp::IsValid(settings.Pelvis, nodesCount) || IsFlat(normal))
        return 0.0f;
    LegWarp::Chain legs[2];
    const int32 legsCount = LegWarp::GetLegs(nodes, nodesCount, settings.Pelvis, settings.Legs, legs);
    Vector3 offsets[2];
    for (int32 i = 0; i < legsCount; i++)
        offsets[i] = Vector3(0, GetSlopeHeight(LegWarp::GetModelTransform(nodes, pose, legs[i].DriveNode).Translation, normal), 0);
    return LegWarp::GetPelvisDrop(nodes, pose, legs, legsCount, offsets, settings.MaxPelvisDrop);
}

void SlopeWarp::Warp(const SkeletonNode* nodes, int32 nodesCount, const Settings& settings, const Float3& normal, float pelvisDrop, Transform* pose)
{
    if (!LegWarp::IsValid(settings.Pelvis, nodesCount) || (IsFlat(normal) && pelvisDrop <= 0.0f))
        return;
    LegWarp::Chain legs[2];
    const int32 legsCount = LegWarp::GetLegs(nodes, nodesCount, settings.Pelvis, settings.Legs, legs);
    if (legsCount == 0)
        return;

    // The feet moved straight up or down onto the slope and turned with it, fixed to the ground (they don't follow the pelvis down)
    const Quaternion tilt = IsFlat(normal) ? Quaternion::Identity : Quaternion::FindBetween(Float3::UnitY, normal);
    Transform ikTargets[2], footTargets[2];
    for (int32 i = 0; i < legsCount; i++)
    {
        const LegWarp::Chain& leg = legs[i];
        const Transform foot = LegWarp::GetModelTransform(nodes, pose, leg.Foot);
        const Transform ikFoot = leg.IKFoot != -1 ? LegWarp::GetModelTransform(nodes, pose, leg.IKFoot) : foot;
        const Real height = GetSlopeHeight(ikFoot.Translation, normal);
        ikTargets[i] = ikFoot;
        ikTargets[i].Translation.Y += height;
        ikTargets[i].Orientation = tilt * ikFoot.Orientation;
        ikTargets[i].Orientation.Normalize();
        footTargets[i] = foot;
        footTargets[i].Translation.Y += height;
        footTargets[i].Orientation = tilt * foot.Orientation;
        footTargets[i].Orientation.Normalize();
    }
    LegWarp::Solve(nodes, settings.Pelvis, legs, legsCount, ikTargets, footTargets, pelvisDrop, pose);
}
