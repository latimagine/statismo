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

#ifndef __STATIMO_ITK_STATISTICAL_MODEL_TRANSFORM_H_
#define __STATIMO_ITK_STATISTICAL_MODEL_TRANSFORM_H_

#include "statismo/core/Representer.h"
#include "statismo/core/NonCopyable.h"
#include "statismo/ITK/itkStatisticalModel.h"

#include <itkImage.h>
#include <itkTransform.h>
#include <itkVector.h>

#include <iostream>

/**
 * \defgroup Transforms ITK transforms classes and routines
 */

namespace itk
{

/**
 *
 * \brief Base class that implements the itk transform interface for statistical models
 *
 * Statistical models (shape or deformation models) are often used to model the typical variations within
 * an object class. The StatisticalModelTransformBase implements the standard Transform interface, and thus allows
 * for the use of statistical models within the ITK registration framework.
 *
 * Subclasses will need to implement the TransformPoint method, as its semantics depends on the type of statistical
 * model.
 *
 * \ingroup Transforms
 * \ingroup ITK
 */

template <typename Dataset,
          typename TransformScalarType,
          unsigned int INPUT_DIMENSION,
          unsigned int OUTPUT_DIMENSION = INPUT_DIMENSION>
class ITK_EXPORT StatisticalModelTransformBase
  : public itk::Transform<TransformScalarType, INPUT_DIMENSION, OUTPUT_DIMENSION>
  , public statismo::NonCopyable
{
public:
  /* Standard class using =s. */
  using Self = StatisticalModelTransformBase;
  using Superclass = itk::Transform<TransformScalarType, INPUT_DIMENSION, OUTPUT_DIMENSION>;
  using Pointer = SmartPointer<Self>;
  using ConstPointer = SmartPointer<const Self>;
  using VectorType = vnl_vector<statismo::ScalarType>;
  using MatrixType = vnl_matrix<statismo::ScalarType>;
  /* Parameters Type   */
  using ParametersType = typename Superclass::ParametersType;
  using JacobianType = typename Superclass::JacobianType;
  using ScalarType = typename Superclass::ScalarType;
  using InputPointType = typename Superclass::InputPointType;
  using OutputPointType = typename Superclass::OutputPointType;
  using InputVectorType = typename Superclass::InputVectorType;
  using OutputVectorType = typename Superclass::OutputVectorType;
  using InputVnlVectorType = typename Superclass::InputVnlVectorType;
  using OutputVnlVectorType = typename Superclass::OutputVnlVectorType;
  using InputCovariantVectorType = typename Superclass::InputCovariantVectorType;
  using OutputCovariantVectorType = typename Superclass::OutputCovariantVectorType;
  using RepresenterType = statismo::Representer<Dataset>;
  using StatisticalModelType = itk::StatisticalModel<Dataset>;

  /** Run-time type information (and related methods). */
  itkTypeMacro(StatisticalModelTransformBase, Superclass);

  virtual void
  CopyBaseMembers(StatisticalModelTransformBase * another) const
  {
    another->m_statisticalModel = m_statisticalModel;
    another->m_coeffVector = m_coeffVector;
    another->m_usedNumberCoefficients = m_usedNumberCoefficients;
    another->m_fixedParameters = m_fixedParameters;
    another->m_Parameters = this->m_Parameters;
  }

  itkStaticConstMacro(SpaceDimension, unsigned int, INPUT_DIMENSION);
  itkStaticConstMacro(InputSpaceDimension, unsigned int, INPUT_DIMENSION);
  itkStaticConstMacro(OutputSpaceDimension, unsigned int, OUTPUT_DIMENSION);

  void
  ComputeJacobianWithRespectToParameters(const InputPointType & pt, JacobianType & jacobian) const override;

  virtual void
  SetIdentity();

  void
  SetParameters(const ParametersType &) override;

  const ParametersType &
  GetParameters() const override;

  void
  SetFixedParameters(const ParametersType &) override
  {
    // there no fixed parameters
  }

  const ParametersType &
  GetFixedParameters() const override
  {
    return this->m_fixedParameters;
  };

  /**
   * \brief Convenience method to obtain the current coefficients of the StatisticalModel as a statismo::VectorType.
   * \note The resulting vector is the same as it would be obtained from GetParameters.
   */
  virtual VectorType
  GetCoefficients() const
  {
    return m_coeffVector;
  }

  /**
   * \brief Convenience method to set the coefficients of the underlying StatisticalModel from a statismo::VectorType.
   * \note This has the same effect as calling SetParameters
   */
  virtual void
  SetCoefficients(VectorType & coefficients)
  {
    m_coeffVector = coefficients;
  }

  /**
   * \brief Set the statistical model that defines the valid transformations
   */
  void
  SetStatisticalModel(const StatisticalModelType * model);

  typename StatisticalModelType::ConstPointer
  GetStatisticalModel() const;

  /**
   * \brief Set the number of PCA Coefficients used by the model
   *
   * This parameters has a regularization effect. Setting it to a small value will restrict the possible tranformations
   * to the main modes of variations.
   */
  void
  SetUsedNumberOfCoefficients(unsigned n)
  {
    m_usedNumberCoefficients = n;
  }

  unsigned
  GetUsedNumberOfCoefficients()
  {
    return m_usedNumberCoefficients;
  }

protected:
  StatisticalModelTransformBase();

  void
  PrintSelf(std::ostream & os, Indent indent) const override;

  typename StatisticalModelType::ConstPointer m_statisticalModel;
  VectorType                                  m_coeffVector;
  unsigned                                    m_usedNumberCoefficients;
  ParametersType                              m_fixedParameters;
};


} // namespace itk

#include "itkStatisticalModelTransformBase.hxx"

#endif
