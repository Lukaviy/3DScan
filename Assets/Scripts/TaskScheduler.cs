using System;
using System.Collections;
using System.Collections.Generic;
using System.Threading;
using System.Threading.Tasks;
using UnityEngine;

public class TaskTokenInternal<T>
{
    public Task<T> task;
}

public abstract class TaskTokenBase
{
    public abstract TaskStatus Status { get; }
}

public class TaskToken<T> : TaskTokenBase
{
    public TaskToken(TaskTokenInternal<T> tokeInternal)
    {
        m_tokeInternal = tokeInternal;
    }

    private TaskTokenInternal<T> m_tokeInternal;

    public T Result => m_tokeInternal.task.Result;

    public bool IsCompleted => m_tokeInternal.task?.IsCompleted ?? false;

    public override TaskStatus Status => m_tokeInternal.task?.Status ?? TaskStatus.WaitingToRun;
}

public class TaskScheduler
{
    private int m_maxTasksCount;

    public TaskScheduler(int maxTasksCount)
    {
        m_maxTasksCount = maxTasksCount;
    }

    private abstract class TaskContainerBase
    {
        public abstract TaskTokenBase Token { get; }
        public abstract void Run();
    }

    private class TaskContainer<T> : TaskContainerBase
    {
        private Func<T> m_func;
        private CancellationToken m_cancellationToken;
        private TaskTokenInternal<T> m_tokenInternal;

        public TaskContainer(Func<T> func, CancellationToken cancellationToken, TaskTokenInternal<T> tokenInternal, TaskTokenBase token)
        {
            m_func = func;
            m_cancellationToken = cancellationToken;
            m_tokenInternal = tokenInternal;
            Token = token;
        }

        public override void Run()
        {
            m_tokenInternal.task = Task.Run(m_func, m_cancellationToken);
        }

        public override TaskTokenBase Token { get; }
    }

    private List<TaskTokenBase> m_evaluatingTasks = new List<TaskTokenBase>();
    private Queue<TaskContainerBase> m_taskContainer = new Queue<TaskContainerBase>();

    public TaskToken<T> EnqueueTask<T>(Func<T> func, CancellationToken cancellationToken)
    {
        var tokenInternal = new TaskTokenInternal<T>();
        var token = new TaskToken<T>(tokenInternal);
        m_taskContainer.Enqueue(new TaskContainer<T>(func, cancellationToken, tokenInternal, token));
        return token;
    }

    public void Update()
    {
        m_evaluatingTasks.RemoveAll(x => x.Status is TaskStatus.RanToCompletion or TaskStatus.Faulted);
        while (m_evaluatingTasks.Count < m_maxTasksCount && m_taskContainer.Count > 0)
        {
            var task = m_taskContainer.Dequeue();
            task.Run();
            m_evaluatingTasks.Add(task.Token);
        }
    }
}
