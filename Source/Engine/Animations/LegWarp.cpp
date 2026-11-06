// Copyright (c) Wojciech Figat. All rights reserved.

#include "LegWarp.h"
#include "InverseKinematics.h"
#include "Engine/Core/Math/Math.h"
#include "Engine/Graphics/Models/SkeletonData.h"

namespace
{
    // Squared sine of the angle below which two directions are taken as parallel
    constexpr Real MinBendSineSq = 1e-6f;

    // Sine of the angle between the thigh and the hip-to-foot line: at or below StraightLegSine the leg's bend plane is noise and the knee hinge of the thigh takes over, at or above BentLegSine the input bend plane is used
    constexpr float StraightLegSine = 0.05f;
    constexpr float BentLegSine = 0.15f;

    // The most the pelvis lowers per unit a foot is out of reach. It keeps the drop from jumping as a foot nears the edge of what lowering can reach, the foot stops short instead.
    constexpr Real MaxDropPerStretch = 2.0f;

    Transform GetParentModelTransform(const SkeletonNode* nodes, const Transform* pose, int32 index)
    {
        const int32 parent = nodes[index].ParentIndex;
        return parent >= 0 && parent < index ? LegWarp::GetModelTransform(nodes, pose, parent) : Transform::Identity;
    }

    void SetModelTransform(int32 index, const Transform& parentModel, const Transform& target, Transform* pose)
    {
        const Float3 scale = pose[index].Scale;
        pose[index] = parentModel.WorldToLocal(target);
        pose[index].Orientation.Normalize();
        pose[index].Scale = scale;
    }

    // Gets the leg's knee hinge in thigh space from the reference pose (the knee bends about a fixed axis of the thigh, whichever way the model faces), zero when the reference leg is straight
    Vector3 GetThighHinge(const SkeletonNode* nodes, const LegWarp::Chain& leg)
    {
        const Transform& calf = nodes[leg.Calf].LocalTransform;
        const Vector3 line = calf.LocalToWorld(nodes[leg.Foot].LocalTransform).Translation;
        const Vector3 normal = Vector3::Cross(line, calf.Translation);
        return normal.LengthSquared() > MinBendSineSq * line.LengthSquared() * calf.Translation.LengthSquared() ? normal.GetNormalized() : Vector3::Zero;
    }

    // Gets how far the hip has to lower so it is no further than the reach from the foot (only lowering helps, and only when the foot is below the hip)
    Real GetRequiredDrop(const Vector3& hipToFoot, Real reach)
    {
        const Real lengthSq = hipToFoot.LengthSquared();
        if (lengthSq <= reach * reach)
            return 0;
        const Real up = hipToFoot.Y;
        if (up >= 0)
            return 0;
        const Real groundSq = Math::Max<Real>(lengthSq - up * up, 0);
        const Real limit = (Math::Sqrt(lengthSq) - reach) * MaxDropPerStretch;
        if (groundSq >= reach * reach)
            return Math::Min(-up, limit); // Can't reach from any height, get as close as possible (hip level with the foot)
        return Math::Min(-up - Math::Sqrt(reach * reach - groundSq), limit);
    }

    // Gets how much a foot counts toward the pelvis drop: fully when near the lowest foot, not at all once raised well above it (a swing foot lands wherever the leg can reach, so it must not pull the pelvis down)
    float GetGroundedWeight(Real heightAboveLowest, Real legLength)
    {
        constexpr float GroundedHeight = 0.1f; // In leg lengths
        constexpr float RaisedHeight = 0.3f; // In leg lengths
        if (legLength < ZeroTolerance)
            return 0.0f;
        return 1.0f - Math::Saturate(((float)(heightAboveLowest / legLength) - GroundedHeight) / (RaisedHeight - GroundedHeight));
    }
}

bool LegWarp::IsValid(int32 index, int32 nodesCount)
{
    return index >= 0 && index < nodesCount;
}

Transform LegWarp::GetModelTransform(const SkeletonNode* nodes, const Transform* pose, int32 index)
{
    Transform result = pose[index];
    for (int32 parent = nodes[index].ParentIndex; parent >= 0 && parent < index; parent = nodes[parent].ParentIndex)
    {
        result = pose[parent].LocalToWorld(result);
        index = parent;
    }
    return result;
}

int32 LegWarp::GetLegs(const SkeletonNode* nodes, int32 nodesCount, int32 pelvis, const WarpLeg legs[2], Chain chains[2])
{
    int32 count = 0;
    for (int32 i = 0; i < 2; i++)
    {
        const WarpLeg& leg = legs[i];
        if (!IsValid(leg.Foot, nodesCount) || !IsValid(leg.Thigh, nodesCount))
            continue;
        const int32 calf = nodes[leg.Foot].ParentIndex;
        if (!IsValid(calf, nodesCount) || nodes[calf].ParentIndex != leg.Thigh || leg.Thigh == pelvis)
            continue;
        Chain& chain = chains[count++];
        chain.IKFoot = IsValid(leg.IKFoot, nodesCount) ? leg.IKFoot : -1;
        chain.DriveNode = chain.IKFoot != -1 ? chain.IKFoot : leg.Foot;
        chain.Foot = leg.Foot;
        chain.Calf = calf;
        chain.Thigh = leg.Thigh;
    }
    return count;
}

float LegWarp::GetPelvisDrop(const SkeletonNode* nodes, const Transform* pose, const Chain* legs, int32 legsCount, const Vector3* footOffsets, float maxDrop)
{
    if (maxDrop <= 0.0f)
        return 0.0f;

    // Each leg keeps reaching no further from its thigh than in the input pose, so it keeps its knee bend
    Vector3 hips[2], feet[2];
    Real legLengths[2];
    Real lowest = MAX_Real;
    for (int32 i = 0; i < legsCount; i++)
    {
        const Chain& leg = legs[i];
        const Transform thigh = GetModelTransform(nodes, pose, leg.Thigh);
        const Transform calf = thigh.LocalToWorld(pose[leg.Calf]);
        hips[i] = thigh.Translation;
        feet[i] = calf.LocalToWorld(pose[leg.Foot]).Translation;
        legLengths[i] = Vector3::Distance(calf.Translation, hips[i]) + Vector3::Distance(feet[i], calf.Translation);
        lowest = Math::Min(lowest, feet[i].Y);
    }
    Real drop = 0;
    for (int32 i = 0; i < legsCount; i++)
    {
        const Real reach = Vector3::Distance(feet[i], hips[i]);
        const Real required = GetRequiredDrop(feet[i] + footOffsets[i] - hips[i], reach);
        drop = Math::Max(drop, required * GetGroundedWeight(feet[i].Y - lowest, legLengths[i]));
    }
    return Math::Min((float)drop, maxDrop);
}

void LegWarp::Solve(const SkeletonNode* nodes, int32 pelvis, const Chain* legs, int32 legsCount, const Transform* ikTargets, const Transform* footTargets, float pelvisDrop, Transform* pose)
{
    // The input legs, before anything moves
    Vector3 inputHips[2], inputKnees[2], inputFeet[2], inputHinges[2];
    for (int32 i = 0; i < legsCount; i++)
    {
        const Chain& leg = legs[i];
        const Transform thigh = GetModelTransform(nodes, pose, leg.Thigh);
        const Transform calf = thigh.LocalToWorld(pose[leg.Calf]);
        inputHips[i] = thigh.Translation;
        inputKnees[i] = calf.Translation;
        inputFeet[i] = calf.LocalToWorld(pose[leg.Foot]).Translation;
        inputHinges[i] = Vector3::Transform(GetThighHinge(nodes, leg), thigh.Orientation);
    }

    // Lower the pelvis
    if (pelvisDrop > 0.0f)
    {
        Transform target = GetModelTransform(nodes, pose, pelvis);
        target.Translation.Y -= pelvisDrop;
        SetModelTransform(pelvis, GetParentModelTransform(nodes, pose, pelvis), target, pose);
    }

    // Move the IK feet
    for (int32 i = 0; i < legsCount; i++)
    {
        if (legs[i].IKFoot != -1)
            SetModelTransform(legs[i].IKFoot, GetParentModelTransform(nodes, pose, legs[i].IKFoot), ikTargets[i], pose);
    }

    // Solve each leg onto its foot target
    for (int32 i = 0; i < legsCount; i++)
    {
        const Chain& leg = legs[i];
        const Transform thighParent = GetParentModelTransform(nodes, pose, leg.Thigh);
        Transform thigh = thighParent.LocalToWorld(pose[leg.Thigh]);
        Transform calf = thigh.LocalToWorld(pose[leg.Calf]);
        Transform foot = calf.LocalToWorld(pose[leg.Foot]);
        Vector3 target = footTargets[i].Translation;

        // Stop short of targets out of reach, rather than stretching the leg
        const Real legLength = Vector3::Distance(calf.Translation, thigh.Translation) + Vector3::Distance(foot.Translation, calf.Translation);
        const Vector3 toTarget = target - thigh.Translation;
        const Real toTargetLength = toTarget.Length();
        const Real maxReach = legLength * 0.9999f;
        if (toTargetLength > maxReach)
            target = thigh.Translation + toTarget * (maxReach / toTargetLength);

        // Keep the knee bending the way it did: stay in the input bend plane, on the same side of the hip-to-foot line. Turning the knee along with the line instead would snap it sideways when the line swings a long way (a raised foot near the hip).
        const Vector3 inputKnee = inputKnees[i] - inputHips[i];
        const Vector3 inputLine = inputFeet[i] - inputHips[i];
        const Vector3 newLine = target - thigh.Translation;
        const Vector3 inputNormal = Vector3::Cross(inputLine, inputKnee);
        const Real inputNormalLength = inputNormal.Length();
        const Real inputLengths = inputLine.Length() * inputKnee.Length();
        const float bent = inputLengths > ZeroTolerance ? Math::Saturate(((float)(inputNormalLength / inputLengths) - StraightLegSine) / (BentLegSine - StraightLegSine)) : 0.0f;
        Vector3 bendNormal = inputHinges[i] * (Real)(1.0f - bent);
        if (inputNormalLength > ZeroTolerance)
        {
            // Without a hinge, or with a knee bending against it so they cancel out, go with the input
            const Vector3 inputDirection = inputNormal / inputNormalLength;
            bendNormal += inputDirection * (Real)bent;
            if (bendNormal.LengthSquared() < 0.01f)
                bendNormal = inputDirection;
        }
        const Vector3 bend = Vector3::Cross(bendNormal, newLine);
        const Vector3 pole = thigh.Translation + (bend.LengthSquared() > MinBendSineSq * bendNormal.LengthSquared() * newLine.LengthSquared() ? bend : inputKnee);

        InverseKinematics::SolveTwoBoneIK(thigh, calf, foot, target, pole);
        thigh.Orientation.Normalize();
        calf.Orientation.Normalize();
        foot.Orientation = footTargets[i].Orientation;

        SetModelTransform(leg.Thigh, thighParent, thigh, pose);
        SetModelTransform(leg.Calf, thigh, calf, pose);
        SetModelTransform(leg.Foot, calf, foot, pose);
    }
}
