/*
 * This file is part of the statismo library.
 *
 * Copyright (c) 2019 Laboratory of Medical Information Processing
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * Neither the name of the project's author nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef __STATIMO_CORE_THREAD_POOL_H_
#define __STATIMO_CORE_THREAD_POOL_H_

#include "statismo/core/CommonTypes.h"
#include "statismo/core/NonCopyable.h"
#include "statismo/core/SafeContainer.h"
#include "statismo/core/Exceptions.h"

#include <thread>
#include <mutex>
#include <deque>
#include <future>
#include <type_traits>
#include <memory>

namespace statismo
{

/**
 * \brief Function type erasure utility
 * \note: Implementation taken from "C++ concurrency in action", A. Williams
 *
 * \ingroup Core
 */
class FunctionWrapper : public NonCopyable
{
  struct ImplBase
  {
    virtual void
    Call() = 0;
    virtual ~ImplBase() = default;
  };
  std::unique_ptr<ImplBase> m_impl;
  template <typename F>
  struct Impl : ImplBase
  {
    F f;
    explicit Impl(F && f)
      : f{ std::move(f) }
    {}
    void
    Call() override
    {
      f();
    }
  };

public:
  template <typename F, typename Dummy = std::enable_if_t<!std::is_base_of_v<FunctionWrapper, std::decay_t<F>>>>
  FunctionWrapper(F && f) // NOLINT
    : m_impl{ std::make_unique<Impl<F>>(std::forward<F>(f)) }
  {}

  void
  operator()()
  {
    m_impl->Call();
  }
  FunctionWrapper() = default;
  FunctionWrapper(FunctionWrapper && other) noexcept
    : m_impl{ std::move(other.m_impl) }
  {}
  FunctionWrapper &
  operator=(FunctionWrapper && other) noexcept
  {
    m_impl = std::move(other.m_impl);
    return *this;
  }
};

class WorkStealingQueue : public NonCopyable
{
public:
  using TaskType = FunctionWrapper;

  void
  Push(TaskType data)
  {
    std::lock_guard<std::mutex> lock{ m_mut };
    m_queue.push_front(std::move(data));
  }

  bool
  Empty() const
  {
    std::lock_guard<std::mutex> lock{ m_mut };
    return m_queue.empty();
  }

  bool
  TryPop(TaskType & t)
  {
    std::lock_guard<std::mutex> lock{ m_mut };

    if (m_queue.empty())
    {
      return false;
    }

    t = std::move(m_queue.front());
    m_queue.pop_front();
    return true;
  }

  bool
  TrySteal(TaskType & t)
  {
    std::lock_guard<std::mutex> lock{ m_mut };

    if (m_queue.empty())
    {
      return false;
    }

    t = std::move(m_queue.back());
    m_queue.pop_back();
    return true;
  }

private:
  std::deque<TaskType> m_queue;
  mutable std::mutex   m_mut;
};

/**
 * \brief Thread wrapper to ensure RAII
 *
 * \ingroup Core
 */
class RaiiThread : public NonCopyable
{
public:
  enum class Action
  {
    NONE,
    JOIN,
    DETACH
  };
  RaiiThread() = default;
  explicit RaiiThread(std::thread && t, Action a = Action::JOIN)
    : m_thread{ std::move(t) }
    , m_action{ a }
  {}
  virtual ~RaiiThread() // NOLINT
  {
    if (m_action == Action::JOIN)
    {
      m_thread.join();
    }
    else if (m_action == Action::DETACH)
    {
      m_thread.detach();
    }
  }

  RaiiThread(RaiiThread && other) noexcept
    : m_thread{ std::move(other.m_thread) }
  {
    std::swap(other.m_action, m_action);
  }
  RaiiThread &
  operator=(RaiiThread && other) noexcept
  {
    m_thread = std::move(other.m_thread);
    std::swap(other.m_action, m_action);
    return *this;
  }

  std::thread &
  Get()
  {
    return m_thread;
  }

private:
  std::thread m_thread;
  Action      m_action{ Action::NONE };
};

/*
 * Thread pool used for computation intensive algorithm
 *
 * Implementation taken from "C++ concurrency in action", A. Williams
 */
/**
 * \brief Thread pool
 * \note Implementation taken from "C++ concurrency in action", A. Williams
 * and modified for our needs
 *
 * \ingroup Core
 */
class ThreadPool final : public NonCopyable
{
public:
  enum class WaitingMode
  {
    YIELD = 0,
    WAIT_FOR = 1
  };

  using TaskType = FunctionWrapper;

  explicit ThreadPool(unsigned maxThreads, WaitingMode m, unsigned waitTime)
    : m_waitMode{ m }
    , m_waitTime{ waitTime }
  {
    const auto kThreadCount = std::min(std::thread::hardware_concurrency(), maxThreads);

    for (std::remove_cv_t<decltype(kThreadCount)> i = 0; i < kThreadCount; ++i)
    {
      m_queues.push_back(std::make_unique<WorkStealingQueue>());
    }

    try
    {
      for (std::remove_cv_t<decltype(kThreadCount)> i = 0; i < kThreadCount; ++i)
      {
        m_threads.push_back(RaiiThread{ std::thread{ &ThreadPool::DoThreadJob, this, i } });
      }
    }
    catch (...)
    {
      m_isDone = true;
      throw StatisticalModelException("Failed to create thread pool");
    }
  }

  explicit ThreadPool(unsigned maxThreads = std::numeric_limits<unsigned int>::max())
    : ThreadPool{ maxThreads, WaitingMode::YIELD, 0 }
  {}

  virtual ~ThreadPool() { m_isDone = true; } // NOLINT

  template <typename F>
  std::future<std::invoke_result_t<F>>
  Submit(F t)
  {
    std::packaged_task<std::invoke_result_t<F>()> task{ t };
    std::future<std::invoke_result_t<F>>          res{ task.get_future() };

    if (m_localQueue != nullptr)
    {
      m_localQueue->Push(std::move(task));
    }
    else
    {
      m_poolQueue.Push(std::move(task));
    }

    return res;
  }

private:
  void
  DoThreadJob(std::size_t idx)
  {
    m_tid = idx;
    m_localQueue = m_queues[m_tid].get();

    while (!m_isDone)
    {
      RunPendingTask();
    }
  }

  bool
  PopTaskFromLocalQueue(TaskType & t)
  {
    return m_localQueue->TryPop(t);
  }

  bool
  PopTaskFromPoolQueue(TaskType & t)
  {
    return m_poolQueue.TryPop(t);
  }

  bool
  PopTaskFromOtherLocalQueues(TaskType & t)
  {
    for (std::size_t i = 0; i < m_queues.size(); ++i)
    {
      if (m_queues[(m_tid + i + 1) % m_queues.size()]->TrySteal(t))
      {
        return true;
      }
    }

    return false;
  }

  void
  RunPendingTask()
  {
    TaskType t;

    if (PopTaskFromLocalQueue(t) || PopTaskFromPoolQueue(t) || PopTaskFromOtherLocalQueues(t))
    {
      t();
    }
    else
    {
      if (m_waitMode == WaitingMode::YIELD)
      {
        std::this_thread::yield();
      }
      else
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(m_waitTime));
      }
    }
  }

  std::atomic_bool                                m_isDone{ false };
  WaitingMode                                     m_waitMode{ WaitingMode::YIELD };
  unsigned                                        m_waitTime{ 0 };
  SafeQueue<TaskType>                             m_poolQueue;
  std::vector<std::unique_ptr<WorkStealingQueue>> m_queues;
  std::vector<RaiiThread>                         m_threads;

  static thread_local WorkStealingQueue * m_localQueue;
  static thread_local std::size_t         m_tid;
};

inline thread_local WorkStealingQueue * ThreadPool::m_localQueue = nullptr; // NOLINT
inline thread_local std::size_t         ThreadPool::m_tid = 0;              // NOLINT

} // namespace statismo

#endif