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

#ifndef __STATIMO_ITK_STATISTICAL_SHAPE_MODEL_TRANSFORM_H_
#define __STATIMO_ITK_STATISTICAL_SHAPE_MODEL_TRANSFORM_H_

#include "statismo/ITK/itkStandardImageRepresenter.h"
#include "statismo/ITK/itkStatisticalModel.h"
#include "statismo/ITK/itkStatisticalModelTransformBase.h"

#include <itkImage.h>
#include <itkVector.h>

#include <iostream>

namespace itk
{

/**
 *
 * \brief An itk transform that allows for deformations defined by a given Statistical Shape Model.
 *
 *
 * \ingroup Transforms
 */
template <class TRepresenter, class TScalarType, unsigned int TDimension>
class ITK_EXPORT StatisticalShapeModelTransform
  : public itk::StatisticalModelTransformBase<TRepresenter, TScalarType, TDimension>
{
public:
  /* Standard class using =s. */
  using Self = StatisticalShapeModelTransform;
  using Superclass = itk::StatisticalModelTransformBase<TRepresenter, TScalarType, TDimension>;
  using Pointer = SmartPointer<Self>;
  using ConstPointer = SmartPointer<const Self>;

  itkSimpleNewMacro(Self);
  /** Run-time type information (and related methods). */
  itkTypeMacro(StatisticalShapeModelTransform, Superclass);

  using InputPointType = typename Superclass::InputPointType;
  using OutputPointType = typename Superclass::OutputPointType;
  using RepresenterType = typename Superclass::RepresenterType;

  virtual ::itk::LightObject::Pointer
  CreateAnother() const override
  {
    ::itk::LightObject::Pointer smartPtr;
    Pointer                     another = Self::New().GetPointer();
    this->CopyBaseMembers(another);

    smartPtr = static_cast<Pointer>(another);
    return smartPtr;
  }

  virtual OutputPointType
  TransformPoint(const InputPointType & pt) const override
  {
    return this->m_statisticalModel->DrawSampleAtPoint(this->m_coeffVector, pt);
  }

  StatisticalShapeModelTransform() = default;
};


} // namespace itk

#endif // __ItkStatisticalShapeModelTransform
