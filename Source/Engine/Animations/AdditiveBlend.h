// Copyright (c) Wojciech Figat. All rights reserved.

#pragma once

#include "Engine/Core/Math/Transform.h"

struct SkeletonNode;

/// <summary>
/// Additive pose blending in mesh (model) space: each node's rotation offset is taken and applied in the model's space instead of its parent's.
/// A local space additive turns each node about its own (already animated) parent's axes, so on a chain that the base pose has twisted (eg. a walking spine) the offsets of the nodes down the chain add up along different axes than they were authored on. In mesh space each node's offset turns it about the model's axes, the same whatever the base pose's parents do, which suits aim offsets.
/// </summary>
class FLAXENGINE_API AdditiveBlend
{
public:
    /// <summary>
    /// Adds an additive pose over a base pose with its rotations applied in mesh space.
    /// The additive pose holds each node's offset from the reference (bind) pose: the node's mesh space rotation offset is its additive mesh space rotation over its reference mesh space rotation, and that is put in front of the base pose's mesh space rotation. Translation and scale add in local space (as the local space additive).
    /// </summary>
    /// <param name="skeleton">The skeleton nodes (their parents and their reference local transformations).</param>
    /// <param name="count">The amount of nodes.</param>
    /// <param name="base">The base pose (local node transformations).</param>
    /// <param name="additive">The additive pose (local node transformations): the reference pose with the offsets on it.</param>
    /// <param name="alpha">How much of the offsets to add (0-1).</param>
    /// <param name="result">The output pose (local node transformations). Can't be the same as the inputs.</param>
    /// <param name="reference">The pose the offsets are taken from (local node transformations), or null for the skeleton's reference (bind) pose. Give a pose sampled the same way as the additive one (eg. an aim offset's centre pose) so both are in the same nodes' frames.</param>
    static void MeshSpace(const SkeletonNode* skeleton, int32 count, const Transform* base, const Transform* additive, float alpha, Transform* result, const Transform* reference = nullptr);
};
