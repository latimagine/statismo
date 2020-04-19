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

#ifndef __STATISMO_VTK_STATISMO_OUTPUT_WINDOW_H_
#define __STATISMO_VTK_STATISMO_OUTPUT_WINDOW_H_

#include "statismo/VTK/StatismoVTKExport.h"

#include <vtkOutputWindow.h>

namespace statismo
{
class Logger;

/**
 * \brief Specific output window used to redirect VTK log to Statismo logger
 * \ingroup VTK
 */
class STATISMO_VTK_EXPORT vtkStatismoOutputWindow : public vtkOutputWindow
{
public:
  vtkTypeMacro(vtkStatismoOutputWindow, vtkOutputWindow); // NOLINT

  static vtkStatismoOutputWindow *
  New();

  vtkStatismoOutputWindow(const vtkStatismoOutputWindow &) = delete;
  void
  operator=(const vtkStatismoOutputWindow &) = delete;

  void
  DisplayText(const char * t) override;

  void
  DisplayErrorText(const char * t) override;

  void
  DisplayWarningText(const char * t) override;

  void
  DisplayGenericWarningText(const char * t) override;

  void
  DisplayDebugText(const char * t) override;

  void
  SetLogger(Logger * logger)
  {
    m_redirectLogger = logger;
  }

  Logger *
  GetLogger() const
  {
    return m_redirectLogger;
  }

protected:
  vtkStatismoOutputWindow();
  ~vtkStatismoOutputWindow() override;

private:
  statismo::Logger * m_redirectLogger{ nullptr };
};
} // namespace statismo

#endif