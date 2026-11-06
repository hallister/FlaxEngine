// Copyright (c) Wojciech Figat. All rights reserved.

#pragma once

#include "LegWarp.h"

/// <summary>
/// Slope warping: fits the legs of a pose made on flat ground to a sloped ground plane under the character.
/// Each IK foot moves straight up or down onto the slope (the plane through the model origin with the ground normal) and turns with it. The pelvis lowers so the downhill leg can still reach, and each leg (thigh, calf, foot) is solved onto its moved foot target with two bone IK, the same leg solve as Stride Warp.
/// The up axis is the model Y axis.
/// </summary>
class FLAXENGINE_API SlopeWarp
{
public:
    /// <summary>
    /// The warping setup.
    /// </summary>
    struct Settings
    {
        /// <summary>
        /// The pelvis node. It lowers so the legs can reach (only when it is an ancestor of the thighs). Required.
        /// </summary>
        int32 Pelvis = -1;

        /// <summary>
        /// The legs to warp (left and right). The IK foot (optional) sets where the foot meets the slope.
        /// </summary>
        WarpLeg Legs[2];

        /// <summary>
        /// The most the pelvis may lower (in model units). 0 or less keeps the pelvis where it is, and legs that can't reach their targets stay short of them.
        /// </summary>
        float MaxPelvisDrop = 50.0f;

        /// <summary>
        /// The steepest slope the legs fit to (in degrees). Steeper ground normals are turned back to it.
        /// </summary>
        float MaxSlopeAngle = 45.0f;
    };

    /// <summary>
    /// Gets the ground normal to warp to: the given normal (in model space) weighted by alpha against up, and no steeper than the max slope angle. Up when the normal has no length or points down.
    /// </summary>
    /// <param name="normal">The ground normal (in model space).</param>
    /// <param name="alpha">How much of the slope to apply (0-1).</param>
    /// <param name="maxSlopeAngle">The steepest slope (in degrees).</param>
    /// <returns>The normalized ground normal.</returns>
    static Float3 GetGroundNormal(const Float3& normal, float alpha, float maxSlopeAngle);

    /// <summary>
    /// Gets the height of the slope above a position's point on the flat ground: the plane through the model origin with the ground normal.
    /// </summary>
    /// <param name="position">The position (in model space). Only its X and Z are used.</param>
    /// <param name="normal">The ground normal (normalized, pointing up).</param>
    /// <returns>The height along the model Y axis.</returns>
    static Real GetSlopeHeight(const Vector3& position, const Float3& normal);

    /// <summary>
    /// Checks if the setup can warp anything: the pelvis is set and at least one leg is a valid thigh, calf and foot chain.
    /// </summary>
    static bool CanWarp(const SkeletonNode* nodes, int32 nodesCount, const Settings& settings);

    /// <summary>
    /// Gets how much the pelvis lowers so no grounded leg reaches further from its thigh than it does in the input pose once its foot is on the slope, limited by the settings' maximum pelvis drop. Feet raised well above the lowest foot don't count.
    /// </summary>
    /// <param name="nodes">The skeleton nodes (sorted so parents come before their children).</param>
    /// <param name="nodesCount">The amount of skeleton nodes.</param>
    /// <param name="settings">The warping setup.</param>
    /// <param name="normal">The ground normal (in model space), eg. from GetGroundNormal.</param>
    /// <param name="pose">The pose node transformations (in local space).</param>
    /// <returns>The pelvis drop (in model units, 0 or more).</returns>
    static float GetPelvisDrop(const SkeletonNode* nodes, int32 nodesCount, const Settings& settings, const Float3& normal, const Transform* pose);

    /// <summary>
    /// Warps the pose in place.
    /// </summary>
    /// <param name="nodes">The skeleton nodes (sorted so parents come before their children).</param>
    /// <param name="nodesCount">The amount of skeleton nodes.</param>
    /// <param name="settings">The warping setup.</param>
    /// <param name="normal">The ground normal (in model space), eg. from GetGroundNormal.</param>
    /// <param name="pelvisDrop">How much to lower the pelvis (in model units), eg. from GetPelvisDrop.</param>
    /// <param name="pose">The pose node transformations (in local space), modified in place.</param>
    static void Warp(const SkeletonNode* nodes, int32 nodesCount, const Settings& settings, const Float3& normal, float pelvisDrop, Transform* pose);
};
