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


#ifndef __STATIMO_ITK_MODEL_BUILDER_H_
#define __STATIMO_ITK_MODEL_BUILDER_H_

#include <itkObject.h>
#include <itkObjectFactory.h>

#include "statismo/ITK/itkDataManager.h"
#include "statismo/ITK/itkStatisticalModel.h"

#include "statismo/core/PCAModelBuilder.h"
#include "statismo/ITK/itkConfig.h"
#include "statismo/core/ImplWrapper.h"
#include "statismo/ITK/itkUtils.h"

#include <utility>
#include <functional>

namespace itk
{

/**
 * \brief ITK Wrapper for the statismo::PCAModelBuilder class.
 * \see statismo::PCAModelBuilder for detailed documentation.
 */
template <class Representer>
class PCAModelBuilder
  : public Object
  , public statismo::ImplWrapper<statismo::PCAModelBuilder<Representer>, statismo::SafeInitializer>
{
public:
  typedef PCAModelBuilder          Self;
  typedef Object                   Superclass;
  typedef SmartPointer<Self>       Pointer;
  typedef SmartPointer<const Self> ConstPointer;

  using ImplType =
    typename statismo::ImplWrapper<statismo::PCAModelBuilder<Representer>, statismo::SafeInitializer>::ImplType;

  itkNewMacro(Self);
  itkTypeMacro(PCAModelBuilder, Object);

  typedef statismo::DataManager<Representer>         DataManagerType;
  typedef typename DataManagerType::DataItemListType DataItemListType;

  typedef typename ImplType::EigenValueMethod EigenValueMethod;

  PCAModelBuilder() {}

  virtual ~PCAModelBuilder() {}

  typename StatisticalModel<Representer>::Pointer
  BuildNewModel(DataItemListType DataItemList,
                float            noiseVariance,
                bool             computeScores = true,
                EigenValueMethod method = ImplType::JacobiSVD)
  {
    auto model_statismo = this->CallForwardImplTrans(
      ExceptionHandler{ *this }, &ImplType::BuildNewModel, DataItemList, noiseVariance, computeScores, method);

    typename StatisticalModel<Representer>::Pointer model_itk = StatisticalModel<Representer>::New();
    model_itk->SetStatismoImplObj(std::move(model_statismo));
    return model_itk;
  }
};


} // namespace itk

#endif /* ITKMODELBUILDER_H_ */