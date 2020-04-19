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

#include "statismo/core/LoggerMultiHandlersThreaded.h"
#include "statismo/core/CommonTypes.h"
#include "statismo/core/ThreadPool.h"
#include "statismo/core/SafeContainer.h"

#include <algorithm>
#include <atomic>
#include <cassert>
#include <condition_variable>
#include <mutex>
#include <numeric>
#include <queue>
#include <thread>
#include <iostream>
#include <vector>
#include <chrono>
#include <ctime>

// default handlers
namespace statismo
{

std::string
DefaultFormatter::operator()(const LogEntry & entry, LogLevel level) const
{
  auto t = std::chrono::system_clock::to_time_t(entry.time);
  auto tstr = std::string(std::ctime(&t));
  tstr = (tstr.pop_back(), "<" + tstr + ">");
  auto mod = (entry.module.empty()) ? " " : " " + entry.module + ": ";
  return tstr + " <" + std::to_string(level) + ">" + mod + entry.log + " (" + entry.file + ":" + entry.line + ")";
}

void
StdOutLogWriter::operator()(const std::string & log) const
{
  std::cout << log << std::endl;
}

FileLogWriter::FileLogWriter(const std::string & f)
  : m_file{ f }
{}

void
FileLogWriter::operator()(const std::string & log)
{
  m_file << log << std::endl;
}
} // namespace statismo

// logger private impl
namespace statismo
{

struct LoggerMultiHandlersThreaded::LoggerMultiHandlersThreadedImpl
{
  using LogEntryWithLevelType = std::pair<LogEntry, LogLevel>;
  SafeQueue<LogEntryWithLevelType> entries;
  HandlerListType                  handlers;
  LevelListType                    levels;
  std::unique_ptr<RaiiThread>      loggerThread;
  std::mutex                       loggerMut;
  std::mutex                       startMut;
  std::condition_variable          startCondV;
  LogLevel                         defaultLevel{ LogLevel::LOG_INFO };
  std::atomic_bool                 doRun{ false };
  bool                             isStarted{ false };
  bool                             isBackground{ false };

  LoggerMultiHandlersThreadedImpl(LogLevel level, bool isThreaded)
    : defaultLevel{ level }
    , isBackground{ isThreaded }
  {}

  explicit LoggerMultiHandlersThreadedImpl(bool isThreaded)
    : isBackground{ isThreaded }
  {}
  LoggerMultiHandlersThreadedImpl(const LoggerMultiHandlersThreadedImpl &) = delete;
  LoggerMultiHandlersThreadedImpl &
  operator=(const LoggerMultiHandlersThreadedImpl &) = delete;
  LoggerMultiHandlersThreadedImpl(LoggerMultiHandlersThreadedImpl &&) = delete;
  LoggerMultiHandlersThreadedImpl &
  operator=(LoggerMultiHandlersThreadedImpl &&) = delete;

  void
  BackgroundProcess()
  {
    // start processing log entries
    {
      std::lock_guard<std::mutex> lck{ loggerMut };
      isStarted = true;
    }
    startCondV.notify_one();

    LogEntryWithLevelType entry;
    while (doRun || !entries.Empty())
    {
      if (entries.TryPop(entry))
      {
        WriteEntry(entry);
      }
      else
      {
        using namespace std::chrono_literals;
        std::this_thread::sleep_for(25ms);
      }
    }
  }

  void
  WriteEntry(const LogEntryWithLevelType & entry)
  {
    if (doRun)
    {
      for (std::size_t i = 0; i < handlers.size(); ++i)
      {
        if (static_cast<int>(levels[i]) <= static_cast<int>(entry.second))
        {
          handlers[i]->Write(entry.first, entry.second);
        }
      }
    }
  }

  ~LoggerMultiHandlersThreadedImpl() { doRun = false; }
};

} // namespace statismo

// logger public impl
namespace statismo
{
LoggerMultiHandlersThreaded::LoggerMultiHandlersThreaded(LogLevel level, bool isThreaded)
  : m_impl{ std::make_unique<LoggerMultiHandlersThreadedImpl>(level, isThreaded) }
{}

LoggerMultiHandlersThreaded::LoggerMultiHandlersThreaded(HandlerWithLevelListType && list, bool isThreaded)
  : m_impl{ std::make_unique<LoggerMultiHandlersThreadedImpl>(isThreaded) }
{
  AddHandlerList(std::move(list));
}

LoggerMultiHandlersThreaded::LoggerMultiHandlersThreaded(HandlerListType && list, LogLevel level, bool isThreaded)
  : LoggerMultiHandlersThreaded(level, isThreaded)
{
  AddHandlerList(std::move(list));
}

LoggerMultiHandlersThreaded::LoggerMultiHandlersThreaded(HandlerType && handler, LogLevel level, bool isThreaded)
  : LoggerMultiHandlersThreaded(level, isThreaded)
{
  AddHandler(std::move(handler), level);
}

LoggerMultiHandlersThreaded::~LoggerMultiHandlersThreaded() = default;

LogHandleType
LoggerMultiHandlersThreaded::AddHandler(HandlerType && handler, LogLevel level)
{
  auto handle = reinterpret_cast<LogHandleType>(handler.get());

  m_impl->handlers.push_back(std::move(handler));
  m_impl->levels.push_back(level);

  assert(m_impl->handlers.size() == m_impl->levels.size());

  return handle;
}

std::vector<LogHandleType>
LoggerMultiHandlersThreaded::AddHandlerList(HandlerListType && list)
{
  return AddHandlerList(std::move(list), m_impl->defaultLevel);
}

std::vector<LogHandleType>
LoggerMultiHandlersThreaded::AddHandlerList(HandlerWithLevelListType && list)
{
  auto mixedList = std::move(list);

  std::vector<LogHandleType> handles(mixedList.size());
  for (auto & item : mixedList)
  {
    handles.push_back(reinterpret_cast<LogHandleType>(item.first.get()));
    m_impl->handlers.push_back(std::move(item.first));
    m_impl->levels.push_back(item.second);
  }

  assert(m_impl->handlers.size() == m_impl->levels.size());

  return handles;
}

std::vector<LogHandleType>
LoggerMultiHandlersThreaded::AddHandlerList(HandlerListType && list, LogLevel level)
{
  auto l = std::move(list);

  std::vector<LogHandleType> handles(l.size());
  for (const auto & item : l)
  {
    handles.push_back(reinterpret_cast<LogHandleType>(item.get()));
  }

  m_impl->levels.insert(std::end(m_impl->levels), l.size(), level);
  m_impl->handlers.insert(
    std::end(m_impl->handlers), std::make_move_iterator(std::begin(l)), std::make_move_iterator(std::end(l)));

  assert(m_impl->handlers.size() == m_impl->levels.size());

  return handles;
}

void
LoggerMultiHandlersThreaded::RemoveHandler(LogHandleType handle)
{
  auto match = std::find_if(std::cbegin(m_impl->handlers), std::cend(m_impl->handlers), [=](const auto & smart) {
    return reinterpret_cast<LogHandleType>(smart.get()) == handle;
  });

  if (match != std::cend(m_impl->handlers))
  {
    m_impl->levels.erase(std::cbegin(m_impl->levels) + std::distance(std::cbegin(m_impl->handlers), match));
    m_impl->handlers.erase(match);
  }

  assert(m_impl->handlers.size() == m_impl->levels.size());
}

void
LoggerMultiHandlersThreaded::RemoveHandlerList(const std::vector<LogHandleType> & handles)
{
  for (auto h : handles)
  {
    RemoveHandler(h);
  }
}

std::size_t
LoggerMultiHandlersThreaded::GetHandlersCount() const
{
  return m_impl->handlers.size();
}

void
LoggerMultiHandlersThreaded::SetDefaultLevel(LogLevel level)
{
  m_impl->defaultLevel = level;
}

void
LoggerMultiHandlersThreaded::Start()
{
  m_impl->doRun = true;

  if (m_impl->isBackground)
  {
    // launch the processing of log entries in a background thread
    m_impl->loggerThread =
      std::make_unique<RaiiThread>(std::thread(&LoggerMultiHandlersThreadedImpl::BackgroundProcess, m_impl.get()));

    // return when the thread is started
    // use of unique_lock allows easy mutex management
    std::unique_lock<std::mutex> lock(m_impl->loggerMut);
    // conditional variable is associated with the mutex
    m_impl->startCondV.wait(lock, [&]() { return m_impl->isStarted; });
  }
}

void
LoggerMultiHandlersThreaded::Stop()
{
  m_impl->doRun = false;

  if (m_impl->isBackground)
  {
    m_impl->loggerThread.reset(nullptr);
  }
}

void
LoggerMultiHandlersThreaded::Log(LogEntry && entry, LogLevel level)
{
  if (m_impl->isBackground)
  {
    m_impl->entries.Push(std::make_pair(std::move(entry), level));
  }
  else
  {
    // We make writing log thread safe
    std::lock_guard<std::mutex> lock(m_impl->loggerMut);
    m_impl->WriteEntry(std::make_pair(std::move(entry), level));
  }
}

void
LoggerMultiHandlersThreaded::Log(const LogEntry & entry, LogLevel level)
{
  auto tmpEntry = entry;
  Log(std::move(tmpEntry), level);
}
} // namespace statismo