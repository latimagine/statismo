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


#ifndef __STATIMO_ITK_CONDITONAL_MODEL_BUILDER_H_
#define __STATIMO_ITK_CONDITONAL_MODEL_BUILDER_H_

#include "statismo/core/ImplWrapper.h"
#include "statismo/core/ConditionalModelBuilder.h"
#include "statismo/ITK/itkDataManager.h"
#include "statismo/ITK/itkStatisticalModel.h"
#include "statismo/ITK/itkConfig.h"
#include "statismo/ITK/itkUtils.h"

#include <itkObject.h>
#include <itkObjectFactory.h>

#include <functional>

namespace itk
{

/**
 * \brief ITK wrapper for statismo::ConditionalModelBuilder class
 *
 * \sa statismo::ConditionalModelBuilder for detailed documentation
 * \ingroup ModelBuilders
 * \ingroup ITK
 */
template <typename Representer>
class ConditionalModelBuilder
  : public Object
  , public statismo::ImplWrapper<statismo::ConditionalModelBuilder<Representer>, statismo::SafeInitializer>
{
public:
  using Self = ConditionalModelBuilder;
  using Superclass = Object;
  using Pointer = SmartPointer<Self>;
  using ConstPointer = SmartPointer<const Self>;

  itkNewMacro(Self);
  itkTypeMacro(ConditionalModelBuilder, Object);

  using DataManagerType = statismo::DataManager<Representer>;
  using SampleDataStructureListType = typename DataManagerType::SampleDataStructureListType;
  using ImplType =
    typename statismo::ImplWrapper<statismo::ConditionalModelBuilder<Representer>, statismo::SafeInitializer>::ImplType;

  typename StatisticalModel<Representer>::Pointer
  BuildNewModel(
    SampleDataStructureListType                                                              SampleDataStructureList,
    const typename statismo::ConditionalModelBuilder<Representer>::SurrogateTypeVectorType & surrogateTypes,
    const typename statismo::ConditionalModelBuilder<Representer>::CondVariableValueVectorType & conditioningInfo,
    float                                                                                        noiseVariance,
    double                                                                                       modelVarianceRetained)
  {
    auto statismoModel = this->CallForwardImplTrans(statismo::itk::ExceptionHandler{ *this },
                                                    &ImplType::BuildNewModel,
                                                    SampleDataStructureList,
                                                    surrogateTypes,
                                                    conditioningInfo,
                                                    noiseVariance,
                                                    modelVarianceRetained);

    auto modelItk = StatisticalModel<Representer>::New();
    modelItk->SetStatismoImplObj(std::move(statismoModel));
    return modelItk;
  }

  virtual void
  SetLogger(statismo::Logger * logger)
  {
    this->CallForwardImpl(&ImplType::SetLogger, logger);
  }
};

} // namespace itk

#endif
