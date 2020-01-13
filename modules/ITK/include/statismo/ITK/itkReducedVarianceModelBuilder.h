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


#ifndef __STATIMO_ITK_REDUCED_VARIANCE_MODEL_BUILDER_H_
#define __STATIMO_ITK_REDUCED_VARIANCE_MODEL_BUILDER_H_

#include "statismo/core/ReducedVarianceModelBuilder.h"
#include "statismo/core/Utils.h"
#include "statismo/core/ImplWrapper.h"
#include "statismo/ITK/itkDataManager.h"
#include "statismo/ITK/itkStatisticalModel.h"
#include "statismo/ITK/itkConfig.h"
#include "statismo/ITK/itkUtils.h"

#include <itkObject.h>

#include <functional>
#include <utility>

namespace itk
{

/**
 * \brief ITK Wrapper for statismo::ReducedVarianceModelBuilder class.
 * \see statismo::ReducedVariance for detailed documentation.
 */
template <class Representer>
class ReducedVarianceModelBuilder
  : public Object
  , public statismo::ImplWrapper<statismo::ReducedVarianceModelBuilder<Representer>, statismo::SafeInitializer>
{
public:
  using Self = ReducedVarianceModelBuilder ;
  using Superclass =  Object                      ;
  using Pointer = SmartPointer<Self>          ;
  using ConstPointer = SmartPointer<const Self>    ;
  using ImplType = typename statismo::ImplWrapper<statismo::ReducedVarianceModelBuilder<Representer>,
                                                  statismo::SafeInitializer>::ImplType;

  itkNewMacro(Self);
  itkTypeMacro(ReducedVarianceModelBuilder, Object);

  ReducedVarianceModelBuilder() = default;

  virtual ~ReducedVarianceModelBuilder() = default;

  typename StatisticalModel<Representer>::Pointer
  BuildNewModelWithLeadingComponents(const StatisticalModel<Representer> * model, unsigned numberOfPrincipalComponents)
  {
    const auto* statismoModel = model->GetStatismoImplObj();

    auto newStatismoModel = this->CallForwardImplTrans(statismo::itk::ExceptionHandler{ *this },
                                                         &ImplType::BuildNewModelWithLeadingComponents,
                                                         statismoModel,
                                                         numberOfPrincipalComponents);
    auto itkModel = StatisticalModel<Representer>::New();
    itkModel->SetStatismoImplObj(std::move(newStatismoModel));
    return itkModel;
  }

  typename StatisticalModel<Representer>::Pointer
  BuildNewModelWithVariance(const StatisticalModel<Representer> * model, double totalVariance)
  {
    const auto * statismoModel = model->GetStatismoImplObj();
    auto                                            newStatismoModel = this->CallForwardImplTrans(
      statismo::itk::ExceptionHandler{ *this }, &ImplType::BuildNewModelWithVariance, statismoModel, totalVariance);

    auto itkModel = StatisticalModel<Representer>::New();
    itkModel->SetStatismoImplObj(std::move(newStatismoModel));
    return itkModel;
  }

  [[deprecated]] typename StatisticalModel<Representer>::Pointer
  BuildNewModelFromModel(const StatisticalModel<Representer> * model, double totalVariance)
  {
    auto * statismoModel = model->GetStatismoImplObj();
    auto                                      newStatismoModel = this->CallForwardImplTrans(
      statismo::itk::ExceptionHandler{ *this }, &ImplType::BuildNewModelFromModel, statismoModel, totalVariance);

    auto itkModel = StatisticalModel<Representer>::New();
    itkModel->SetStatismoImplObj(std::move(newStatismoModel));
    return itkModel;
  }
};


} // namespace itk

#endif
