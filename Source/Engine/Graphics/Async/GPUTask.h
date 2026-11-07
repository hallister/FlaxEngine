// Copyright (c) Wojciech Figat. All rights reserved.

#pragma once

#include "Engine/Threading/Task.h"
#include "Engine/Platform/Platform.h"
#include "GPUTasksContext.h"

class GPUResource;

/// <summary>
/// Describes GPU work object.
/// </summary>
class GPUTask : public Task
{
    friend GPUTasksContext;
    friend class GPUTasksManager;

public:
    /// <summary>
    /// Describes GPU work type
    /// </summary>
    DECLARE_ENUM_EX_4(Type, byte, 0, Custom, CopyResource, UploadTexture, UploadBuffer);

    /// <summary>
    /// Describes GPU work result value
    /// </summary>
    DECLARE_ENUM_4(Result, Ok, Failed, MissingResources, MissingData);

private:
    /// <summary>
    /// Task type
    /// </summary>
    Type _type;

    byte _syncLatency;

    /// <summary>
    /// Synchronization point when async task has been done
    /// </summary>
    GPUSyncPoint _syncPoint;

    /// <summary>
    /// The context that performed this task, it should synchronize it.
    /// </summary>
    GPUTasksContext* _context;

    /// <summary>
    /// References that keep this task alive: one released when the ObjectsRemovalService deletes the task, and one held by the GPUTasksManager while the task is in its queue or buffers.
    /// An ended task can stay in the manager for longer than its time to live (eg. behind a long queue when frames are slow), so it's freed only once the manager has let go of it too.
    /// </summary>
    volatile int64 _refs = 1;

    void ReleaseRef()
    {
        if (Platform::InterlockedDecrement(&_refs) == 0)
            Task::OnDeleteObject();
    }

protected:
    /// <summary>
    /// Initializes a new instance of the <see cref="GPUTask"/> class.
    /// </summary>
    /// <param name="type">The type.</param>
    /// <param name="syncLatency">Amount of frames until async operation is synced with GPU.</param>
    GPUTask(const Type type, byte syncLatency = GPU_ASYNC_LATENCY)
        : _type(type)
        , _syncLatency(syncLatency)
        , _syncPoint(0)
        , _context(nullptr)
    {
    }

public:
    /// <summary>
    /// Gets a task type.
    /// </summary>
    FORCE_INLINE Type GetType() const
    {
        return _type;
    }

    /// <summary>
    /// Gets work synchronization start point
    /// </summary>
    FORCE_INLINE GPUSyncPoint GetSyncStart() const
    {
        return _syncPoint;
    }

    /// <summary>
    /// Gets work finish synchronization point
    /// </summary>
    FORCE_INLINE GPUSyncPoint GetSyncPoint() const
    {
        return _syncPoint + _syncLatency;
    }

public:
    /// <summary>
    /// Checks if operation is syncing
    /// </summary>
    FORCE_INLINE bool IsSyncing() const
    {
        return IsRunning() && _syncPoint != 0;
    }

public:
    /// <summary>
    /// Executes this task.
    /// </summary>
    /// <param name="context">The context.</param>
    void Execute(GPUTasksContext* context);

    /// <summary>
    /// Action fired when asynchronous operation has been synchronized with a GPU
    /// </summary>
    void Sync()
    {
        if (_context != nullptr)
        {
            ASSERT(IsSyncing());

            // Sync and finish
            _context = nullptr;
            OnSync();
            OnFinish();
        }
    }

    /// <summary>
    /// Cancels the task results synchronization.
    /// </summary>
    void CancelSync()
    {
        ASSERT(IsSyncing());

        // Rollback state and cancel
        _context = nullptr;
        SetState(TaskState::Queued);
        Cancel();
    }

protected:
    virtual Result run(GPUTasksContext* context) = 0;

    virtual void OnSync()
    {
    }

public:
    // [Task]
    String ToString() const override;

public:
    // [Object]
    void OnDeleteObject() override
    {
        ReleaseRef();
    }

protected:
    // [Task]
    void Enqueue() override;

    bool Run() override
    {
        return true;
    }

    void OnCancel() override;
};
