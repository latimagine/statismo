/*
 * Copyright (c) 2015 University of Basel
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

#ifndef __STATISMO_FITTING_UTILS_H_
#define __STATISMO_FITTING_UTILS_H_

#include <itkCommand.h>

#include <iostream>
#include <vector>

#ifdef _WIN32
#  include <io.h>
#  define DUP(f) _dup(f)
#  define DUP2(f, newf) _dup2(f, newf)
#  define FILENO(f) _fileno(f)
#  define FOPEN(f, mode) fopen_s(f, mode)
#  define CLOSE(f) _close(f)
#else
#  include <unistd.h>
#  define DUP(fd) dup(fd)
#  define DUP2(fd, newfd) dup2(fd, newfd)
#  define FILENO(f) fileno(f)
#  define FOPEN(f, mode) fopen(f, mode)
#  define CLOSE(f) close(f)
#endif

namespace statismo::cli
{

// The optimizer may print something despite me not wanting it to print anything. It is thus that I manually silence the
// console output when it's not me printing.
class ConsoleOutputSilencer
{
private:
  int    m_oldStdOutDescriptor;
  int    m_oldStdErrDescriptor;
  int    m_nullDescriptor;
  FILE * m_nullFile;
  bool   m_isOutputEnabled;

  void
  FlushAll()
  {
    fflush(stdout);
    fflush(stderr);
    fflush(m_nullFile);
  }

public:
  ~ConsoleOutputSilencer()
  {
    if (m_isOutputEnabled == false)
    {
      EnableOutput();
    }
    CLOSE(m_nullDescriptor);
  }

  ConsoleOutputSilencer()
  {
#ifdef _WIN32
    fopen_s(&m_nullFile, "NUL", "w");
#else
    m_nullFile = fopen("/dev/null", "w");
#endif
    m_oldStdOutDescriptor = DUP(FILENO(stdout));
    m_oldStdErrDescriptor = DUP(FILENO(stderr));
    m_nullDescriptor = FILENO(m_nullFile);
    m_isOutputEnabled = true;
  }

  void
  DisableOutput()
  {
    if (m_isOutputEnabled == false)
    {
      return;
    }
    m_isOutputEnabled = false;
    FlushAll();
    DUP2(m_nullDescriptor, FILENO(stdout));
    DUP2(m_nullDescriptor, FILENO(stderr));
  }

  void
  EnableOutput()
  {
    if (m_isOutputEnabled == true)
    {
      return;
    }
    m_isOutputEnabled = true;
    FlushAll();
    DUP2(m_oldStdOutDescriptor, FILENO(stdout));
    DUP2(m_oldStdErrDescriptor, FILENO(stderr));
  }
};

template <class OptimizerType>
class IterationStatusObserver : public ::itk::Command
{
public:
  using Self = IterationStatusObserver<OptimizerType>;
  using Superclass = ::itk::Command;
  using Pointer = ::itk::SmartPointer<Self>;
  using OptimizerPointer = const OptimizerType *;

  itkNewMacro(Self);

  void
  Execute(::itk::Object * caller, const ::itk::EventObject & event) override
  {
    Execute(static_cast<const ::itk::Object *>(caller), event);
  }

  void
  Execute(const ::itk::Object * object, const ::itk::EventObject & event) override
  {
    auto optimizer = dynamic_cast<OptimizerPointer>(object);

    if (::itk::IterationEvent().CheckEvent(&event) == false || !optimizer)
    {
      return;
    }

    if (m_coSilencer)
    {
      m_coSilencer->EnableOutput();
    }
    std::cout << "Iteration: " << ++m_iterNo;
    std::cout << "; Value: " << optimizer->GetCachedValue();
    std::cout << "; Current Parameters: " << optimizer->GetCachedCurrentPosition() << std::endl;
    if (m_coSilencer)
    {
      m_coSilencer->DisableOutput();
    }
  }

  void
  SetConsoleSilencer(statismo::cli::ConsoleOutputSilencer * cos)
  {
    m_coSilencer = cos;
  }

protected:
  IterationStatusObserver() = default;
  virtual ~IterationStatusObserver() = default;

private:
  int                                    m_iterNo{ 0 };
  statismo::cli::ConsoleOutputSilencer * m_coSilencer{ nullptr };
};


template <class OptimizerType>
static void
InitializeOptimizer(typename OptimizerType::Pointer        optimizer,
                    unsigned                               numberOfIterations,
                    unsigned                               numberOfModelComponents,
                    unsigned                               totalNumberOfOptimizationParameters,
                    bool                                   doPrintFittingInformation,
                    statismo::cli::ConsoleOutputSilencer * coSilencer)
{
  constexpr unsigned uNumberOfRigid2DtransformComponents = 3;
  constexpr unsigned uNumberOfRigid3DtransformComponents = 6;

  optimizer->SetMaximumNumberOfFunctionEvaluations(numberOfIterations);
  optimizer->MinimizeOn();

  if (doPrintFittingInformation == true)
  {
    using ObserverType = IterationStatusObserver<OptimizerType>;
    auto obs = ObserverType::New();
    obs->SetConsoleSilencer(coSilencer);
    optimizer->AddObserver(::itk::IterationEvent(), obs);
  }

  unsigned uNrOfRotationComponents{ 0 };
  unsigned uNrOfTranslationComponents{ 0 };
  switch (totalNumberOfOptimizationParameters - numberOfModelComponents)
  {
    case uNumberOfRigid3DtransformComponents:
      uNrOfRotationComponents = 3;
      uNrOfTranslationComponents = 3;
      break;
    case uNumberOfRigid2DtransformComponents:
      uNrOfRotationComponents = 1;
      uNrOfTranslationComponents = 2;
      break;
    case 0:
      break;
    default:
      itkGenericExceptionMacro(<< "Unknown Transform. Can't scale for that one.")
  }

  if (numberOfModelComponents != totalNumberOfOptimizationParameters)
  {
    const double dModelParamScale = 3;
    const double dRotationScale = 0.1;
    const double dTranslationScale = 1;
    // set the scales of the optimizer, to compensate for potentially different scales of translation, rotation and
    // shape parameters
    typename OptimizerType::ScalesType scales(totalNumberOfOptimizationParameters);
    unsigned                           count = 0;

    for (unsigned i = 0; i < numberOfModelComponents; ++i, ++count)
    {
      scales[count] = 1.0 / (dModelParamScale);
    }
    for (unsigned i = 0; i < uNrOfRotationComponents; ++i, ++count)
    {
      scales[count] = 1.0 / (dRotationScale);
    }
    for (unsigned i = 0; i < uNrOfTranslationComponents; ++i, ++count)
    {
      scales[count] = 1.0 / (dTranslationScale);
    }
    optimizer->SetScales(scales);
  }
}
} // namespace statismo::cli

#endif