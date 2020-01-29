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

#ifndef __STATIMO_ITK_STATISTICAL_MODEL_TRANSFORM_HXX_
#define __STATIMO_ITK_STATISTICAL_MODEL_TRANSFORM_HXX_

#include "statismo/ITK/itkStatisticalModelTransformBase.h"


namespace itk
{

/*!
 * Constructor with default arguments.
 */
template <typename Dataset, typename TransformScalarType, unsigned int INPUT_DIMENSION, unsigned int OUTPUT_DIMENSION>
StatisticalModelTransformBase<Dataset, TransformScalarType, INPUT_DIMENSION, OUTPUT_DIMENSION>::
  StatisticalModelTransformBase()
  : Superclass(0)
  , // we don't know the number of parameters at this point.
  m_statisticalModel(nullptr)
  , m_coeffVector(0)
  , m_usedNumberCoefficients(std::numeric_limits<unsigned>::max())
{ // something large
  itkDebugMacro(<< "Constructor MorphableModelTransform()");

  this->m_fixedParameters.SetSize(0);
}

template <typename Dataset, typename TransformScalarType, unsigned int INPUT_DIMENSION, unsigned int OUTPUT_DIMENSION>
void
StatisticalModelTransformBase<Dataset, TransformScalarType, INPUT_DIMENSION, OUTPUT_DIMENSION>::SetStatisticalModel(
  const StatisticalModelType * model)
{
  itkDebugMacro(<< "Setting statistical model ");
  m_statisticalModel = model;

  this->m_Parameters.SetSize(model->GetNumberOfPrincipalComponents());
  this->m_Parameters.Fill(0.0);

  this->m_coeffVector.set_size(model->GetNumberOfPrincipalComponents());
}

template <typename Dataset, typename TransformScalarType, unsigned int INPUT_DIMENSION, unsigned int OUTPUT_DIMENSION>
typename StatisticalModelTransformBase<Dataset, TransformScalarType, INPUT_DIMENSION, OUTPUT_DIMENSION>::
  StatisticalModelType::ConstPointer
  StatisticalModelTransformBase<Dataset, TransformScalarType, INPUT_DIMENSION, OUTPUT_DIMENSION>::GetStatisticalModel()
    const
{
  itkDebugMacro(<< "Getting statistical model ");
  return m_statisticalModel;
}

template <typename Dataset, typename TransformScalarType, unsigned int INPUT_DIMENSION, unsigned int OUTPUT_DIMENSION>
void
StatisticalModelTransformBase<Dataset, TransformScalarType, INPUT_DIMENSION, OUTPUT_DIMENSION>::SetIdentity()
{
  itkDebugMacro(<< "Setting Identity");

  for (unsigned i = 0; i < this->GetNumberOfParameters(); i++)
  {
    this->m_coeffVector[i] = 0;
  }

  this->Modified();
}

template <typename Dataset, typename TransformScalarType, unsigned int INPUT_DIMENSION, unsigned int OUTPUT_DIMENSION>
void
StatisticalModelTransformBase<Dataset, TransformScalarType, INPUT_DIMENSION, OUTPUT_DIMENSION>::SetParameters(
  const ParametersType & parameters)
{
  itkDebugMacro(<< "Setting parameters " << parameters);

  for (unsigned int i = 0; i < std::min(m_usedNumberCoefficients, static_cast<unsigned>(this->GetNumberOfParameters()));
       i++)
  {
    m_coeffVector[i] = parameters[i];
  }
  for (unsigned int i = std::min(m_usedNumberCoefficients, static_cast<unsigned>(this->GetNumberOfParameters()));
       i < this->GetNumberOfParameters();
       i++)
  {
    m_coeffVector[i] = 0;
  }

  // Modified is always called since we just have a pointer to the
  // parameters and cannot know if the parameters have changed.
  this->Modified();

  itkDebugMacro(<< "After setting parameters ");
}

template <typename Dataset, typename TransformScalarType, unsigned int INPUT_DIMENSION, unsigned int OUTPUT_DIMENSION>
const typename StatisticalModelTransformBase<Dataset, TransformScalarType, INPUT_DIMENSION, OUTPUT_DIMENSION>::
  ParametersType &
  StatisticalModelTransformBase<Dataset, TransformScalarType, INPUT_DIMENSION, OUTPUT_DIMENSION>::GetParameters() const
{
  itkDebugMacro(<< "Getting parameters ");

  for (unsigned int i = 0; i < this->GetNumberOfParameters(); i++)
  {
    this->m_Parameters[i] = this->m_coeffVector[i];
  }
  itkDebugMacro(<< "After getting parameters " << this->m_Parameters);

  return this->m_Parameters;
}

template <typename Dataset, typename TransformScalarType, unsigned int INPUT_DIMENSION, unsigned int OUTPUT_DIMENSION>
void
StatisticalModelTransformBase<Dataset, TransformScalarType, INPUT_DIMENSION, OUTPUT_DIMENSION>::
  ComputeJacobianWithRespectToParameters(const InputPointType & pt, JacobianType & jacobian) const
{
  jacobian.SetSize(OutputSpaceDimension, m_statisticalModel->GetNumberOfPrincipalComponents());
  jacobian.Fill(0);

  const MatrixType & statModelJacobian = m_statisticalModel->GetJacobian(pt);

  for (unsigned i = 0; i < statModelJacobian.rows(); i++)
  {
    for (unsigned j = 0; j < std::min(m_usedNumberCoefficients, static_cast<unsigned>(this->GetNumberOfParameters()));
         j++)
    {
      jacobian[i][j] = statModelJacobian[i][j];
    }
  }

  itkDebugMacro(<< "Jacobian with MM:\n" << jacobian);
  itkDebugMacro(<< "After GetMorphableModelJacobian:"
                << "\nJacobian = \n"
                << jacobian);
}

template <typename Dataset, typename TransformScalarType, unsigned int INPUT_DIMENSION, unsigned int OUTPUT_DIMENSION>
void
StatisticalModelTransformBase<Dataset, TransformScalarType, INPUT_DIMENSION, OUTPUT_DIMENSION>::PrintSelf(
  std::ostream & os,
  Indent         indent) const
{
  Superclass::PrintSelf(os, indent);
}

} // namespace itk

#endif
