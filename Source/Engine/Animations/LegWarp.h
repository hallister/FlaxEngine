// Copyright (c) Wojciech Figat. All rights reserved.

#pragma once

#include "Engine/Core/Math/Transform.h"

struct SkeletonNode;

/// <summary>
/// A leg of a leg warp (Stride Warp, Slope Warp). Node indices are skeleton node indices, -1 when unused.
/// </summary>
struct WarpLeg
{
    /// <summary>
    /// The IK foot node (eg. ik_foot_l). Its position drives the warp and it moves to the warped foot position. When unused, the foot itself drives the warp.
    /// </summary>
    int32 IKFoot = -1;

    /// <summary>
    /// The foot node (eg. foot_l). Its parent is the leg's joint (calf).
    /// </summary>
    int32 Foot = -1;

    /// <summary>
    /// The thigh node (eg. thigh_l). Must be the parent of the foot's parent, otherwise the leg isn't solved.
    /// </summary>
    int32 Thigh = -1;
};

/// <summary>
/// The leg solve shared by the leg warps: each warp moves the feet to new targets, and this lowers the pelvis so the legs can still reach and solves each leg (thigh, calf, foot) onto its target with two bone IK, keeping the knee bend. The up axis is the model Y axis.
/// </summary>
namespace LegWarp
{
    /// <summary>
    /// A leg that can be solved: the foot's parent is the calf and the calf's parent is the thigh.
    /// </summary>
    struct Chain
    {
        int32 DriveNode; // The IK foot, or the foot when the leg has no IK foot
        int32 IKFoot;
        int32 Foot;
        int32 Calf;
        int32 Thigh;
    };

    bool IsValid(int32 index, int32 nodesCount);

    /// <summary>
    /// Gets a node's model space transformation by walking up its ancestors (only the few nodes a warp needs, not the whole skeleton).
    /// </summary>
    Transform GetModelTransform(const SkeletonNode* nodes, const Transform* pose, int32 index);

    /// <summary>
    /// Gets the legs that can be solved, at most two.
    /// </summary>
    /// <returns>The amount of legs written to chains.</returns>
    int32 GetLegs(const SkeletonNode* nodes, int32 nodesCount, int32 pelvis, const WarpLeg legs[2], Chain chains[2]);

    /// <summary>
    /// Gets how much the pelvis lowers so no grounded leg, its foot moved by its offset, reaches further from its thigh than it does in the input pose, limited by maxDrop. Feet raised well above the lowest foot don't count.
    /// </summary>
    float GetPelvisDrop(const SkeletonNode* nodes, const Transform* pose, const Chain* legs, int32 legsCount, const Vector3* footOffsets, float maxDrop);

    /// <summary>
    /// Lowers the pelvis, moves the IK feet onto their targets and solves each leg's foot onto its target (the foot takes the target's model space rotation). Targets out of reach stop short rather than stretch the leg.
    /// </summary>
    void Solve(const SkeletonNode* nodes, int32 pelvis, const Chain* legs, int32 legsCount, const Transform* ikTargets, const Transform* footTargets, float pelvisDrop, Transform* pose);
}
