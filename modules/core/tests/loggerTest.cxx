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
#include "StatismoUnitTest.h"
#include "statismo/core/Exceptions.h"

#include "statismo/core/LoggerMultiHandlersThreaded.h"

#include <iterator>

using namespace statismo;

namespace
{
unsigned    g_logCount{ 0 };
std::string g_log;

struct LogHandler
{
  void
  operator()(const std::string & l)
  {
    g_log = l;
    g_logCount++;
  }
};

struct Formatter
{
  std::string
  operator()(const LogEntry & entry, LogLevel) const
  {
    return entry.log;
  }
};

int
TestMain()
{
  g_logCount = 0;
  g_log = "";
  LoggerMultiHandlersThreaded logger{ LogLevel::LOG_FATAL }; // default logger will have fatal limit
  logger.Start();

  STATISMO_ASSERT_EQ(logger.GetHandlersCount(), 0U);
  STATISMO_ASSERT_EQ(g_logCount, 0U);

  auto debugHander =
    logger.AddHandler(std::make_unique<BasicLogHandler>(LogHandler(), Formatter()), LogLevel::LOG_DEBUG);

  STATISMO_ASSERT_EQ(logger.GetHandlersCount(), 1U);

  logger.Log(LogEntry{ "test1", "test.cpp", "666" }, LogLevel::LOG_DEBUG);
  STATISMO_ASSERT_EQ(g_logCount, 1U);
  STATISMO_ASSERT_EQ(g_log, "test1");

  LoggerMultiHandlersThreaded::HandlerWithLevelListType loggerList;
  loggerList.emplace_back(std::make_unique<BasicLogHandler>(LogHandler(), Formatter()), LogLevel::LOG_INFO);
  loggerList.emplace_back(std::make_unique<BasicLogHandler>(LogHandler(), Formatter()), LogLevel::LOG_WARNING);
  logger.AddHandlerList(std::move(loggerList));

  STATISMO_ASSERT_EQ(logger.GetHandlersCount(), 3U);
  logger.Log(LogEntry{ "test2", "test.cpp", "666" }, LogLevel::LOG_DEBUG);
  STATISMO_ASSERT_EQ(g_logCount, 2U);
  STATISMO_ASSERT_EQ(g_log, "test2");

  logger.Log(LogEntry{ "test3", "test.cpp", "666" }, LogLevel::LOG_INFO);
  STATISMO_ASSERT_EQ(g_logCount, 4U);

  logger.Log(LogEntry{ "test4", "test.cpp", "666" }, LogLevel::LOG_WARNING);
  STATISMO_ASSERT_EQ(g_logCount, 7U);

  LoggerMultiHandlersThreaded::HandlerListType loggerList2;
  loggerList2.emplace_back(std::make_unique<BasicLogHandler>(LogHandler(), Formatter()));
  // should be a log with LOG_FATAL default threshold
  logger.AddHandlerList(std::move(loggerList2));

  STATISMO_ASSERT_EQ(logger.GetHandlersCount(), 4U);

  logger.Log(LogEntry{ "test5", "test.cpp", "666" }, LogLevel::LOG_ERROR);
  STATISMO_ASSERT_EQ(g_logCount, 10U);
  logger.Log(LogEntry{ "test6", "test.cpp", "666" }, LogLevel::LOG_FATAL);
  STATISMO_ASSERT_EQ(g_logCount, 14U);

  LoggerMultiHandlersThreaded::HandlerListType loggerList3;
  loggerList3.emplace_back(std::make_unique<BasicLogHandler>(LogHandler(), Formatter()));
  auto debugHanderList = logger.AddHandlerList(std::move(loggerList3), LogLevel::LOG_DEBUG);
  STATISMO_ASSERT_EQ(logger.GetHandlersCount(), 5U);

  logger.Log(LogEntry{ "test7", "test.cpp", "666" }, LogLevel::LOG_DEBUG);
  STATISMO_ASSERT_EQ(g_logCount, 16U);

  // Remove some handler
  logger.RemoveHandler(debugHander);
  logger.Log(LogEntry{ "test7", "test.cpp", "666" }, LogLevel::LOG_DEBUG);
  STATISMO_ASSERT_EQ(g_logCount, 17U);

  logger.RemoveHandlerList(debugHanderList);
  logger.Log(LogEntry{ "test8", "test.cpp", "666" }, LogLevel::LOG_DEBUG);
  STATISMO_ASSERT_EQ(g_logCount, 17U);

  logger.Stop();

  logger.Log(LogEntry{ "test9", "test.cpp", "666" }, LogLevel::LOG_FATAL);
  STATISMO_ASSERT_EQ(g_logCount, 17U);

  logger.Start();

  logger.Log(LogEntry{ "test9", "test.cpp", "666" }, LogLevel::LOG_FATAL);
  STATISMO_ASSERT_EQ(g_logCount, 20U);

  return EXIT_SUCCESS;
}

int
TestCtor2()
{
  g_logCount = 0;
  g_log = "";
  LoggerMultiHandlersThreaded::HandlerWithLevelListType loggerList;
  loggerList.emplace_back(std::make_unique<BasicLogHandler>(LogHandler(), Formatter()), LogLevel::LOG_INFO);
  loggerList.emplace_back(std::make_unique<BasicLogHandler>(LogHandler(), Formatter()), LogLevel::LOG_WARNING);

  LoggerMultiHandlersThreaded logger{ std::move(loggerList) };
  STATISMO_ASSERT_EQ(logger.GetHandlersCount(), 2U);
  logger.Start();
  logger.Log(LogEntry{ "test1", "test.cpp", "666" }, LogLevel::LOG_DEBUG);
  STATISMO_ASSERT_EQ(g_logCount, 0U);

  logger.Log(LogEntry{ "test2", "test.cpp", "666" }, LogLevel::LOG_INFO);
  STATISMO_ASSERT_EQ(g_logCount, 1U);

  logger.Log(LogEntry{ "test3", "test.cpp", "666" }, LogLevel::LOG_WARNING);
  STATISMO_ASSERT_EQ(g_logCount, 3U);
  logger.Stop();

  return EXIT_SUCCESS;
}

int
TestCtor3()
{
  g_logCount = 0;
  g_log = "";
  LoggerMultiHandlersThreaded::HandlerListType loggerList;
  loggerList.emplace_back(std::make_unique<BasicLogHandler>(LogHandler(), Formatter()));

  LoggerMultiHandlersThreaded logger{ std::move(loggerList), LogLevel::LOG_WARNING };
  STATISMO_ASSERT_EQ(logger.GetHandlersCount(), 1U);
  logger.Start();
  logger.Log(LogEntry{ "test1", "test.cpp", "666" }, LogLevel::LOG_INFO);
  STATISMO_ASSERT_EQ(g_logCount, 0U);
  logger.Log(LogEntry{ "test2", "test.cpp", "666" }, LogLevel::LOG_WARNING);
  STATISMO_ASSERT_EQ(g_logCount, 1U);
  logger.Stop();

  return EXIT_SUCCESS;
}

int
TestFileHandler()
{
  g_logCount = 0;
  g_log = "";

  FileLogWriter logHandler{ "statismoLogs.log" };

  LoggerMultiHandlersThreaded logger{ std::make_unique<BasicLogHandler>(std::ref(logHandler), Formatter()),
                                      LogLevel::LOG_INFO };

  STATISMO_ASSERT_EQ(logger.GetHandlersCount(), 1U);

  logger.Start();

  logger.Log(LogEntry{ "test1", "test.cpp", "666" }, LogLevel::LOG_DEBUG);
  logger.Log(LogEntry{ "test2", "test.cpp", "666" }, LogLevel::LOG_INFO);
  logger.Log(LogEntry{ "test3", "test.cpp", "666" }, LogLevel::LOG_WARNING);

  logger.Stop();

  std::ifstream logFile{ "statismoLogs.log" };

  std::vector<std::string> logs;
  std::copy(
    std::istream_iterator<std::string>(logFile), std::istream_iterator<std::string>(), std::back_inserter(logs));

  STATISMO_ASSERT_EQ(logs.size(), 2U);
  STATISMO_ASSERT_EQ(logs[0], "test2");
  STATISMO_ASSERT_EQ(logs[1], "test3");

  return EXIT_SUCCESS;
}
} // namespace

/**
 * This basic test case covers the test of logger classes used in the
 * framework
 */
int loggerTest([[maybe_unused]] int argc, [[maybe_unused]] char * argv[]) // NOLINT
{
  auto res = statismo::Translate([]() {
    return statismo::test::RunAllTests("loggerTest",
                                       { { "TestMain", TestMain },
                                         { "TestCtor2", TestCtor2 },
                                         { "TestCtor3", TestCtor3 },
                                         { "TestFileHandler", TestFileHandler } });
  });

  return !CheckResultAndAssert(res, EXIT_SUCCESS);
}