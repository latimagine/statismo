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

#ifndef __STATIMO_CORE_LOGGER_H_
#define __STATIMO_CORE_LOGGER_H_

#include "statismo/core/NonCopyable.h"

#include <chrono>
#include <string>

// Utility macros used inside the framework for classes
// with a logger

// clang-format off
#ifdef STATISMO_ENABLE_LOG
#define STATISMO_LOG(str, level) if(this->GetLogger()) this->GetLogger()->Log(statismo::LogEntry{(str), __FILE__, std::to_string(__LINE__)}, level)
#define STATISMO_LOG_DEBUG(str) STATISMO_LOG(str, statismo::LogLevel::LOG_DEBUG)
#define STATISMO_LOG_INFO(str) STATISMO_LOG(str, statismo::LogLevel::LOG_INFO)
#define STATISMO_LOG_WARNING(str) STATISMO_LOG(str, statismo::LogLevel::LOG_WARNING)
#define STATISMO_LOG_ERROR(str) STATISMO_LOG(str, statismo::LogLevel::LOG_ERROR)
#define STATISMO_LOG_FATAL(str) STATISMO_LOG(str, statismo::LogLevel::LOG_FATAL)
#else
#define STATISMO_LOG(level, str)
#define STATISMO_LOG_DEBUG(str)
#define STATISMO_LOG_INFO(str)
#define STATISMO_LOG_WARNING(str)
#define STATISMO_LOG_ERROR(str)
#define STATISMO_LOG_FATAL(str)
#endif
// clang-format on

namespace statismo
{

/**
 * \brief Log level
 * \ingroup Core
 */
enum class LogLevel
{
  LOG_DEBUG = 0, /**< Debug log for very detailed information */
  LOG_INFO,      /**< Info log for standard information */
  LOG_WARNING,   /**< Warning log for important information */
  LOG_ERROR,     /**< Error log for catchable/recoverable errors */
  LOG_FATAL      /**< Error fatal for end-of-program errors */
};

/**
 * \brief Log entry
 * \ingroup Core
 */
struct LogEntry
{
  std::string                                        log;
  std::string                                        file = "";
  std::string                                        line = "";
  std::string                                        module = "";
  std::chrono::time_point<std::chrono::system_clock> time = std::chrono::system_clock::now();
};

/**
 * \brief Base abstract class for loggers
 * \ingroup Core
 */
class Logger : public NonCopyable
{

public:
  /**
   * \brief Log an entry
   */
  virtual void
  Log(LogEntry && entry, LogLevel level) = 0;
  virtual void
  Log(const LogEntry & entry, LogLevel level) = 0;
};

} // namespace statismo

namespace std
{

inline std::string
to_string(statismo::LogLevel level) // NOLINT
{
  switch (level)
  {
    case statismo::LogLevel::LOG_DEBUG:
      return "DEBUG";
    case statismo::LogLevel::LOG_INFO:
      return "INFO";
    case statismo::LogLevel::LOG_WARNING:
      return "WARN";
    case statismo::LogLevel::LOG_ERROR:
      return "ERROR";
    case statismo::LogLevel::LOG_FATAL:
      return "FATAL";
    default:
      return "UNKNOWN";
  }
}
} // namespace std

#endif
