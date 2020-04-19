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

#ifndef __STATIMO_CORE_LOGGER_MULTI_HANDLERS_THREADED_H_
#define __STATIMO_CORE_LOGGER_MULTI_HANDLERS_THREADED_H_

#include "statismo/core/CommonTypes.h"
#include "statismo/core/Logger.h"
#include "statismo/core/StatismoCoreExport.h"

#include <string>
#include <fstream>
#include <functional>
#include <memory>
#include <vector>

namespace statismo
{

using LogHandleType = uintptr_t;

/**
 * \brief Base abstract class for log handlers creation.
 *
 * A log handler is responsible for writing a log entry in
 * a self-defined location.
 *
 * \ingroup Core
 */
class LogHandler : public NonCopyable
{
public:
  /**
   * \brief Write log entry
   * \param entry log entry to be written
   * \param level log level for this entry
   */
  virtual void
  Write(const LogEntry & entry, LogLevel level) = 0;
};

/**
 * \brief Basic concrete implementation used to inject specific handlers
 *        and formatters as lightweight policies
 * \ingroup Core
 */
class BasicLogHandler : public LogHandler
{
public:
  using IOPolicyType = std::function<void(const std::string &)>;
  using FormatPolicyType = std::function<std::string(const LogEntry & entry, LogLevel level)>;

  /**
   * \brief Ctor
   * \param io IO policy in the form of a callable object
   * \param format Format policy in the form of a callable object
   */
  template <typename IOPolicyType, typename FormatPolicyType>
  BasicLogHandler(IOPolicyType && io, FormatPolicyType && format)
    : m_ioPolicy{ std::forward<IOPolicyType>(io) }
    , m_formatPolicy{ std::forward<FormatPolicyType>(format) }
  {}

  void
  Write(const LogEntry & entry, LogLevel level) override
  {
    m_ioPolicy(m_formatPolicy(entry, level));
  }

private:
  IOPolicyType     m_ioPolicy;
  FormatPolicyType m_formatPolicy;
};

/**
 * \brief Specific writer to write to standard output
 * \ingroup Core
 */
struct STATISMO_CORE_EXPORT StdOutLogWriter
{
  void
  operator()(const std::string & log) const;
};

/**
 * \brief Specific writer to write to file
 *
 * This version can be used with BasicLogHandler
 * by using std::ref if its life cycle allows it.
 *
 * If it's not possible, use CallableWrapper to wrap it.
 *
 * \ingroup Core
 */
struct FileLogWriter
{
  STATISMO_CORE_EXPORT explicit FileLogWriter(const std::string & f)
    : file{ f }
  {}

  STATISMO_CORE_EXPORT void
  operator()(const std::string & log);

private:
  std::ofstream file;
};

/**
 * \brief Default formatter
 *
 * The output format is: time LEVEL module log file line
 *
 * \ingroup Core
 */
struct STATISMO_CORE_EXPORT DefaultFormatter
{
  std::string
  operator()(const LogEntry & entry, LogLevel level) const;
};

/**
 * \brief Logger class with background threading capabilities and
 * able to handle multiple log handlers.
 *
 * This class is responsible for all the log mechanism except writing
 * and formatting dedicated to handlers registered in the logger.
 *
 * The logger dispatches a log to specific handlers according to the
 * log level. To know if a log should be passed or not, the logger checks:
 * - its base level of filtering
 * - the specific level of filtering bound to a given log handler
 *
 * Logging can be done in background if parallelism is set.
 *
 * \ingroup Core
 */
class LoggerMultiHandlersThreaded : public Logger
{
public:
  using HandlerType = std::unique_ptr<LogHandler>;
  using HandlerListType = std::vector<HandlerType>;
  using LevelListType = std::vector<LogLevel>;
  using HandlerWithLevelType = std::pair<HandlerType, LogLevel>;
  using HandlerWithLevelListType = std::vector<HandlerWithLevelType>;

  /**
   * \brief Create a logger with base level of filtering
   * \param level base level (only entry with level greater or equal will be processed)
   * \param isThreaded glag set to true if logging should be performed in
   *                       backgound
   */
  STATISMO_CORE_EXPORT explicit LoggerMultiHandlersThreaded(LogLevel level, bool isThreaded = false);

  /**
   * \brief Create a logger from a detailed handler list
   * \param list handlers list with their specific base log level
   * \param isThreaded flag set to true if logging should be performed in
   *                       backgound
   */
  STATISMO_CORE_EXPORT explicit LoggerMultiHandlersThreaded(HandlerWithLevelListType && list, bool isThreaded = false);

  /**
   * \brief Create a logger from a simple handler list
   * \param list Handlers list
   * \param level Base level for all the handlers in the list
   * \param isThreaded Flag set to true if logging should be performed in
   *                       backgound
   */
  STATISMO_CORE_EXPORT explicit LoggerMultiHandlersThreaded(HandlerListType && list,
                                                            LogLevel           level,
                                                            bool               isThreaded = false);

  /**
   * \brief Create a logger with a single handler
   * \param handler log handler
   * \param level base level for the log handler
   * \param isThreaded glag set to true if logging should be performed in
   *                       backgound
   */
  STATISMO_CORE_EXPORT explicit LoggerMultiHandlersThreaded(HandlerType && handler,
                                                            LogLevel       level,
                                                            bool           isThreaded = false);

  STATISMO_CORE_EXPORT ~LoggerMultiHandlersThreaded(); // NOLINT

  /**
   * \brief Add a simple handler
   * \param handler handler
   * \param level base level for this handler
   * \return handle that can be used to remove a handler
   */
  STATISMO_CORE_EXPORT LogHandleType
                       AddHandler(HandlerType && handler, LogLevel level = LogLevel::LOG_INFO);

  /**
   * \brief Add simple handler list
   * \param list handlers list
   * \return Ordered list of handles that can be used to remove a handler
   *
   * \note The handlers log filtering level will be set to the class default log level
   */
  STATISMO_CORE_EXPORT std::vector<LogHandleType>
                       AddHandlerList(HandlerListType && list);

  /**
   * \brief Add detailed handler list
   * \param list handlers list with their specific base log level
   * \return Ordered list of handles that can be used to remove a handler
   */
  STATISMO_CORE_EXPORT std::vector<LogHandleType>
                       AddHandlerList(HandlerWithLevelListType && list);

  /**
   * \brief Add a simple handler list
   * \param list handlers list
   * \param level base level for all the handlers in the list
   * \return Ordered list of handles that can be used to remove a handler
   */
  STATISMO_CORE_EXPORT std::vector<LogHandleType>
                       AddHandlerList(HandlerListType && list, LogLevel level);

  /**
   * \brief Remove single handler
   * \param handle log handle
   */
  STATISMO_CORE_EXPORT void
  RemoveHandler(LogHandleType handle);

  /**
   * \brief Remove handler list
   * \param handles list of log handles
   */
  STATISMO_CORE_EXPORT void
  RemoveHandlerList(const std::vector<LogHandleType> & handles);

  /**
   * \brief Return the number of active handler
   */
  STATISMO_CORE_EXPORT std::size_t
                       GetHandlersCount() const;

  /**
   * \brief Set base logging level
   */
  STATISMO_CORE_EXPORT void
  SetDefaultLevel(LogLevel level);

  /**
   * \brief Start logger
   */
  STATISMO_CORE_EXPORT void
  Start();

  /**
   * \brief Stop logger explicitly
   * \note RAII is ensured within the logger and stop is called on
   *       destruction
   */
  STATISMO_CORE_EXPORT void
  Stop();

  /**
   * \brief Log an entry
   * \pre Start must be called
   */
  STATISMO_CORE_EXPORT void
  Log(LogEntry && entry, LogLevel level) override;
  STATISMO_CORE_EXPORT void
  Log(const LogEntry & entry, LogLevel level) override;

private:
  // pimpl idiom implementation
  struct LoggerMultiHandlersThreadedImpl;
  std::unique_ptr<LoggerMultiHandlersThreadedImpl> m_impl;
};
} // namespace statismo

#endif
