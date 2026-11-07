// Copyright (c) Wojciech Figat. All rights reserved.

#include "Engine/Animations/AdditiveBlend.h"
#include "Engine/Core/Math/Math.h"
#include "Engine/Core/Math/Quaternion.h"
#include "Engine/Graphics/Models/SkeletonData.h"
#include <ThirdParty/catch2/catch.hpp>

namespace
{
    constexpr int32 Count = 3;

    // A chain: root, then a, then b, each offset up from its parent, at rest unrotated
    void SetupSkeleton(SkeletonNode* skeleton)
    {
        for (int32 i = 0; i < Count; i++)
        {
            skeleton[i].ParentIndex = i - 1;
            skeleton[i].LocalTransform = Transform(Vector3(0, i == 0 ? 0.0f : 10.0f, 0), Quaternion::Identity);
        }
    }

    Quaternion Mesh(const SkeletonNode* skeleton, const Transform* pose, int32 index)
    {
        Quaternion q = pose[index].Orientation;
        for (int32 parent = skeleton[index].ParentIndex; parent >= 0; parent = skeleton[parent].ParentIndex)
            q = pose[parent].Orientation * q;
        return q;
    }
}

TEST_CASE("AdditiveBlend")
{
    SkeletonNode skeleton[Count];
    SetupSkeleton(skeleton);
    Transform reference[Count], base[Count], additive[Count], result[Count];
    for (int32 i = 0; i < Count; i++)
        reference[i] = skeleton[i].LocalTransform;

    SECTION("No Offset")
    {
        // An additive pose at the reference adds nothing, whatever the base pose
        for (int32 i = 0; i < Count; i++)
        {
            base[i] = reference[i];
            base[i].Orientation = Quaternion::Euler(10.0f * i, 20.0f, -15.0f * i);
            additive[i] = reference[i];
        }
        AdditiveBlend::MeshSpace(skeleton, Count, base, additive, 1.0f, result);
        for (int32 i = 0; i < Count; i++)
            CHECK(Quaternion::AngleBetween(result[i].Orientation, base[i].Orientation) < 0.01f);
    }

    SECTION("Offset In Mesh Space")
    {
        // The base pose turns the middle node about Z; the additive turns the last node 30 degrees about the model's Y.
        // In mesh space the last node ends up turned about the model's Y, not about its turned parent's
        for (int32 i = 0; i < Count; i++)
        {
            base[i] = reference[i];
            additive[i] = reference[i];
        }
        base[1].Orientation = Quaternion::Euler(0, 0, 90.0f);
        const Quaternion offset = Quaternion::Euler(0, 30.0f, 0);
        additive[2].Orientation = offset;
        AdditiveBlend::MeshSpace(skeleton, Count, base, additive, 1.0f, result);
        const Quaternion expected = offset * Mesh(skeleton, base, 2);
        CHECK(Quaternion::AngleBetween(Mesh(skeleton, result, 2), expected) < 0.01f);
        // The middle node had no offset: unchanged
        CHECK(Quaternion::AngleBetween(Mesh(skeleton, result, 1), Mesh(skeleton, base, 1)) < 0.01f);
        // Not what the local space additive gives (the offset about the turned parent's Y)
        const Quaternion local = Mesh(skeleton, base, 1) * offset;
        CHECK(Quaternion::AngleBetween(Mesh(skeleton, result, 2), local) > 1.0f);
    }

    SECTION("Reference Pose")
    {
        // The additive and reference poses sampled with their nodes in other frames than the bind pose's: here the root
        // turned 90 degrees about X (a yaw in the model's space is a turn about the child's own Z then). Offsets taken
        // from the reference pose come out as the yaw they are in the model's space; taken from the bind pose they don't
        const Quaternion frame = Quaternion::Euler(90.0f, 0, 0);
        const Quaternion yaw = Quaternion::Euler(0, 30.0f, 0);
        Transform referencePose[Count];
        for (int32 i = 0; i < Count; i++)
        {
            base[i] = reference[i];
            referencePose[i] = reference[i];
            additive[i] = reference[i];
        }
        referencePose[0].Orientation = frame;
        additive[0].Orientation = frame;
        // The child turned 30 degrees about the model's Y, put in its parent's (the turned root's) space
        additive[1].Orientation = Quaternion::Invert(frame) * yaw * frame;
        AdditiveBlend::MeshSpace(skeleton, Count, base, additive, 1.0f, result, referencePose);
        const Quaternion expected = yaw * Mesh(skeleton, base, 1);
        CHECK(Quaternion::AngleBetween(Mesh(skeleton, result, 1), expected) < 0.01f);
        CHECK(Quaternion::AngleBetween(Mesh(skeleton, result, 0), Mesh(skeleton, base, 0)) < 0.01f);
        // From the bind pose instead, the root's frame comes in as an offset too
        AdditiveBlend::MeshSpace(skeleton, Count, base, additive, 1.0f, result);
        CHECK(Quaternion::AngleBetween(Mesh(skeleton, result, 1), expected) > 1.0f);
    }

    SECTION("Alpha")
    {
        for (int32 i = 0; i < Count; i++)
        {
            base[i] = reference[i];
            additive[i] = reference[i];
        }
        additive[2].Orientation = Quaternion::Euler(0, 40.0f, 0);
        additive[2].Translation += Vector3(4, 0, 0);
        AdditiveBlend::MeshSpace(skeleton, Count, base, additive, 0.0f, result);
        CHECK(Quaternion::AngleBetween(result[2].Orientation, base[2].Orientation) < 0.01f);
        CHECK(Vector3::Distance(result[2].Translation, base[2].Translation) < 0.001f);
        AdditiveBlend::MeshSpace(skeleton, Count, base, additive, 0.5f, result);
        CHECK(Math::Abs(Quaternion::AngleBetween(result[2].Orientation, base[2].Orientation) - 20.0f) < 0.1f);
        CHECK(Math::Abs((float)result[2].Translation.X - 2.0f) < 0.001f);
    }
}
