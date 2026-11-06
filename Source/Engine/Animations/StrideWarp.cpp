// Copyright (c) Wojciech Figat. All rights reserved.

#include "StrideWarp.h"
#include "Engine/Core/Math/Math.h"
#include "Engine/Graphics/Models/SkeletonData.h"

namespace
{
    // Gets the travel direction on the ground plane (model Y up), false when it has no length there
    bool GetGroundDirection(const Float3& direction, Vector3& result)
    {
        result = Vector3((Real)direction.X, 0, (Real)direction.Z);
        const Real length = result.Length();
        if (length < ZeroTolerance)
            return false;
        result /= length;
        return true;
    }
}

Vector3 StrideWarp::GetFootOffset(const Vector3& footPosition, const Vector3& pelvisPosition, const Vector3& direction, float strideScale)
{
    const Real along = Vector3::Dot(footPosition - pelvisPosition, direction);
    return direction * (along * (Real)(strideScale - 1.0f));
}

bool StrideWarp::CanWarp(const SkeletonNode* nodes, int32 nodesCount, const Settings& settings)
{
    LegWarp::Chain legs[2];
    return LegWarp::IsValid(settings.Pelvis, nodesCount) && LegWarp::GetLegs(nodes, nodesCount, settings.Pelvis, settings.Legs, legs) != 0;
}

float StrideWarp::GetPelvisDrop(const SkeletonNode* nodes, int32 nodesCount, const Settings& settings, const Float3& direction, float strideScale, const Transform* pose)
{
    Vector3 dir;
    if (settings.MaxPelvisDrop <= 0.0f || !LegWarp::IsValid(settings.Pelvis, nodesCount) || !GetGroundDirection(direction, dir) || Math::NearEqual(strideScale, 1.0f))
        return 0.0f;
    LegWarp::Chain legs[2];
    const int32 legsCount = LegWarp::GetLegs(nodes, nodesCount, settings.Pelvis, settings.Legs, legs);
    const Vector3 pelvis = LegWarp::GetModelTransform(nodes, pose, settings.Pelvis).Translation;
    Vector3 offsets[2];
    for (int32 i = 0; i < legsCount; i++)
        offsets[i] = GetFootOffset(LegWarp::GetModelTransform(nodes, pose, legs[i].DriveNode).Translation, pelvis, dir, strideScale);
    return LegWarp::GetPelvisDrop(nodes, pose, legs, legsCount, offsets, settings.MaxPelvisDrop);
}

float StrideWarp::Ease(float current, float target, float speed, float deltaTime)
{
    if (speed <= 0.0f || deltaTime <= 0.0f)
        return speed <= 0.0f ? target : current;
    return Math::Lerp(current, target, 1.0f - Math::Exp(-speed * deltaTime));
}

void StrideWarp::Warp(const SkeletonNode* nodes, int32 nodesCount, const Settings& settings, const Float3& direction, float strideScale, float pelvisDrop, Transform* pose)
{
    Vector3 dir;
    if (!LegWarp::IsValid(settings.Pelvis, nodesCount) || !GetGroundDirection(direction, dir))
        return;
    if (Math::NearEqual(strideScale, 1.0f) && pelvisDrop <= 0.0f)
        return;
    LegWarp::Chain legs[2];
    const int32 legsCount = LegWarp::GetLegs(nodes, nodesCount, settings.Pelvis, settings.Legs, legs);
    if (legsCount == 0)
        return;

    // The warped foot targets, fixed to the ground (they don't follow the pelvis down)
    const Vector3 pelvis = LegWarp::GetModelTransform(nodes, pose, settings.Pelvis).Translation;
    Transform ikTargets[2], footTargets[2];
    for (int32 i = 0; i < legsCount; i++)
    {
        const LegWarp::Chain& leg = legs[i];
        const Transform foot = LegWarp::GetModelTransform(nodes, pose, leg.Foot);
        const Transform ikFoot = leg.IKFoot != -1 ? LegWarp::GetModelTransform(nodes, pose, leg.IKFoot) : foot;
        const Vector3 offset = GetFootOffset(ikFoot.Translation, pelvis, dir, strideScale);
        ikTargets[i] = ikFoot;
        ikTargets[i].Translation += offset;
        footTargets[i] = foot;
        footTargets[i].Translation += offset;
    }
    LegWarp::Solve(nodes, settings.Pelvis, legs, legsCount, ikTargets, footTargets, pelvisDrop, pose);
}
