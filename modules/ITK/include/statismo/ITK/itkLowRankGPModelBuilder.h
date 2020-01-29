/*
 * This file is part of the statismo library.
 *
 * Author: Marcel Luethi (marcel.luethi@unibas.ch)
 *
 * Copyright (c) 2011 University of Basel
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

#ifndef __STATIMO_ITK_LOW_RANK_MODEL_BUILDER_H_
#define __STATIMO_ITK_LOW_RANK_MODEL_BUILDER_H_

#include "statismo/core/Kernels.h"
#include "statismo/core/LowRankGPModelBuilder.h"
#include "statismo/core/Representer.h"
#include "statismo/core/ImplWrapper.h"
#include "statismo/ITK/itkStatisticalModel.h"
#include "statismo/ITK/itkConfig.h"
#include "statismo/ITK/itkUtils.h"

#include <itkObject.h>
#include <itkObjectFactory.h>

namespace itk
{

/**
 * \brief ITK Wrapper for statismo::LowRankGPModelBuilder class.
 * \see statismo::LowRankGPModelBuilder for detailed documentation.
 */

template <typename T>
class LowRankGPModelBuilder
  : public Object
  , public statismo::ImplWrapper<statismo::LowRankGPModelBuilder<T>>
{
public:
  using Self = LowRankGPModelBuilder;
  using RepresenterType = statismo::Representer<T>;
  using Superclass = Object;
  using Pointer = SmartPointer<Self>;
  using ConstPointer = SmartPointer<const Self>;
  using ImplType = typename statismo::ImplWrapper<statismo::LowRankGPModelBuilder<T>>::ImplType;

  itkNewMacro(Self);
  itkTypeMacro(LowRankGPModelBuilder, Object);

  using StatisticalModelType = itk::StatisticalModel<T>;
  using MatrixValuedKernelType = statismo::MatrixValuedKernel<typename RepresenterType::PointType>;

  void
  SetRepresenter(const RepresenterType * representer)
  {
    this->SetStatismoImplObj(ImplType::SafeCreate(representer));
  }

  typename StatisticalModelType::Pointer
  BuildNewZeroMeanModel(const MatrixValuedKernelType & kernel,
                        unsigned                       numComponents,
                        unsigned                       numPointsForNystrom = 500) const
  {
    if (!this->m_impl)
    {
      itkExceptionMacro(<< "Model not properly initialized. Maybe you forgot to call SetRepresenter");
    }

    auto itkModel = StatisticalModel<T>::New();
    try
    {
      itkModel->SetStatismoImplObj(this->m_impl->BuildNewZeroMeanModel(kernel, numComponents, numPointsForNystrom));
    }
    catch (const statismo::StatisticalModelException & s)
    {
      itkExceptionMacro(<< s.what());
    }

    return itkModel;
  }

  typename StatisticalModelType::Pointer
  BuildNewModel(typename RepresenterType::DatasetType * mean,
                const MatrixValuedKernelType &          kernel,
                unsigned                                numComponents,
                unsigned                                numPointsForNystrom = 500)
  {
    if (!this->m_impl)
    {
      itkExceptionMacro(<< "Model not properly initialized. Maybe you forgot to call SetRepresenter");
    }

    auto itkModel = StatisticalModel<T>::New();
    try
    {
      itkModel->SetStatismoImplObj(this->m_impl->BuildNewModel(mean, kernel, numComponents, numPointsForNystrom));
    }
    catch (const statismo::StatisticalModelException & s)
    {
      itkExceptionMacro(<< s.what());
    }

    return itkModel;
  }
};

} // namespace itk

#endif
