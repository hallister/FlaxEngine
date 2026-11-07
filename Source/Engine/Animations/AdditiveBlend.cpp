// Copyright (c) Wojciech Figat. All rights reserved.

#include "AdditiveBlend.h"
#include "Engine/Core/Collections/Array.h"
#include "Engine/Core/Math/Math.h"
#include "Engine/Graphics/Models/SkeletonData.h"

namespace
{
    // The mesh space rotation of a node from the local rotations of its chain (parents before children, or worked out on the way)
    const Quaternion& MeshRotation(const SkeletonNode* skeleton, int32 count, int32 index, const Transform* local, Quaternion* mesh, bool* done)
    {
        if (!done[index])
        {
            const int32 parent = skeleton[index].ParentIndex;
            if (parent >= 0 && parent < count && parent != index)
                mesh[index] = MeshRotation(skeleton, count, parent, local, mesh, done) * local[index].Orientation;
            else
                mesh[index] = local[index].Orientation;
            mesh[index].Normalize();
            done[index] = true;
        }
        return mesh[index];
    }

    void MeshRotations(const SkeletonNode* skeleton, int32 count, const Transform* local, Quaternion* mesh, bool* done)
    {
        for (int32 i = 0; i < count; i++)
            done[i] = false;
        for (int32 i = 0; i < count; i++)
            MeshRotation(skeleton, count, i, local, mesh, done);
    }
}

void AdditiveBlend::MeshSpace(const SkeletonNode* skeleton, int32 count, const Transform* base, const Transform* additive, float alpha, Transform* result)
{
    alpha = Math::Saturate(alpha);
    Array<Transform, InlinedAllocation<256>> reference;
    reference.Resize(count);
    for (int32 i = 0; i < count; i++)
        reference[i] = skeleton[i].LocalTransform;
    Array<Quaternion, InlinedAllocation<256>> baseMesh, additiveMesh, referenceMesh, resultMesh;
    baseMesh.Resize(count);
    additiveMesh.Resize(count);
    referenceMesh.Resize(count);
    resultMesh.Resize(count);
    Array<bool, InlinedAllocation<256>> done;
    done.Resize(count);
    MeshRotations(skeleton, count, base, baseMesh.Get(), done.Get());
    MeshRotations(skeleton, count, additive, additiveMesh.Get(), done.Get());
    MeshRotations(skeleton, count, reference.Get(), referenceMesh.Get(), done.Get());

    // Each node's mesh space offset put in front of its base mesh space rotation, at the alpha
    for (int32 i = 0; i < count; i++)
    {
        Quaternion offset = additiveMesh[i] * Quaternion::Invert(referenceMesh[i]);
        offset.Normalize();
        Quaternion::Slerp(Quaternion::Identity, offset, alpha, offset);
        resultMesh[i] = offset * baseMesh[i];
        resultMesh[i].Normalize();
    }

    // Back to local space, under the result's own parents
    for (int32 i = 0; i < count; i++)
    {
        const int32 parent = skeleton[i].ParentIndex;
        Transform& t = result[i];
        t.Orientation = parent >= 0 && parent < count && parent != i ? Quaternion::Invert(resultMesh[parent]) * resultMesh[i] : resultMesh[i];
        t.Orientation.Normalize();
        t.Translation = base[i].Translation + (additive[i].Translation - reference[i].Translation) * alpha;
        t.Scale = base[i].Scale + (additive[i].Scale - reference[i].Scale) * alpha;
    }
}
