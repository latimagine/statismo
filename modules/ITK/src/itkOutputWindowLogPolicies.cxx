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

#include "statismo/ITK/itkOutputWindowLogPolicies.h"

#include <itkOutputWindow.h>

#include <string>

namespace statismo::itk
{

void
OutputWindowLogWriter::operator()(const std::string & log) const
{
  std::size_t pos;
  if ((pos = log.find(std::to_string(LogLevel::LOG_DEBUG))) == 0)
  {
    ::itk::OutputWindow::GetInstance()->DisplayDebugText(
      log.substr(1 + std::to_string(LogLevel::LOG_DEBUG).size()).c_str());
  }
  else if ((pos = log.find(std::to_string(LogLevel::LOG_WARNING))) == 0)
  {
    ::itk::OutputWindow::GetInstance()->DisplayWarningText(
      log.substr(1 + std::to_string(LogLevel::LOG_WARNING).size()).c_str());
  }
  else if ((pos = log.find(std::to_string(LogLevel::LOG_ERROR))) == 0)
  {
    ::itk::OutputWindow::GetInstance()->DisplayErrorText(
      log.substr(1 + std::to_string(LogLevel::LOG_ERROR).size()).c_str());
  }
  else if ((pos = log.find(std::to_string(LogLevel::LOG_FATAL))) == 0)
  {
    ::itk::OutputWindow::GetInstance()->DisplayErrorText(
      log.substr(1 + std::to_string(LogLevel::LOG_FATAL).size()).c_str());
  }
  else if ((pos = log.find(std::to_string(LogLevel::LOG_INFO))) == 0)
  {
    ::itk::OutputWindow::GetInstance()->DisplayText(log.substr(1 + std::to_string(LogLevel::LOG_INFO).size()).c_str());
  }
  else
  {
    ::itk::OutputWindow::GetInstance()->DisplayText(log.c_str());
  }
}

std::string
MessageFormatter::operator()(const LogEntry & entry, LogLevel level) const
{
  std::string itkMsg = std::to_string(level) + " ";
  itkMsg += entry.log;
  itkMsg += "\n";
  itkMsg += ("In " + entry.file + ", line " + entry.line);
  itkMsg += "\n";
  return itkMsg;
}

} // namespace statismo::itk
