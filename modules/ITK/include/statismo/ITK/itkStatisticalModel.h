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

#ifndef __STATIMO_ITK_STATISTICAL_MODEL_H_
#define __STATIMO_ITK_STATISTICAL_MODEL_H_

#include "statismo/core/ModelInfo.h"
#include "statismo/core/Representer.h"
#include "statismo/core/StatisticalModel.h"
#include "statismo/core/ImplWrapper.h"
#include "statismo/ITK/itkConfig.h"
#include "statismo/ITK/itkUtils.h"

#include <itkObject.h>
#include <itkObjectFactory.h>

#include <vnl/vnl_matrix.h>
#include <vnl/vnl_vector.h>

#include <utility>
#include <functional>

namespace itk
{

/**
 * \brief ITK Wrapper for the statismo::StatisticalModel class.
 * \see statismo::StatisticalModel for detailed documentation.
 */
template <class T>
class StatisticalModel
  : public Object
  , public statismo::ImplWrapper<statismo::StatisticalModel<T>>
{
public:
  using Self = StatisticalModel;
  using Superclass = Object;
  using Pointer = SmartPointer<Self>;
  using ConstPointer = SmartPointer<const Self>;
  using ImplType = typename statismo::ImplWrapper<statismo::StatisticalModel<T>>::ImplType;

  itkNewMacro(Self);
  itkTypeMacro(StatisticalModel, Object);

  using RepresenterType =  statismo::Representer<T>;
  using DataItemType =  typename statismo::DataManager<T>::DataItemType ;
  using MatrixType =  vnl_matrix<statismo::ScalarType> ;
  using VectorType = vnl_vector<statismo::ScalarType> ;
  using DatasetPointerType = typename RepresenterType::DatasetPointerType      ;
  using DatasetConstPointerType = typename RepresenterType::DatasetConstPointerType ;
  using ValueType = typename RepresenterType::ValueType ;
  using PointType = typename RepresenterType::PointType ;
  using PointValuePairType = typename statismo::StatisticalModel<T>::PointValuePairType ;
  using PointValueListType = typename statismo::StatisticalModel<T>::PointValueListType ;
  using PointCovarianceMatrixType = typename statismo::StatisticalModel<T>::PointCovarianceMatrixType        ;
  using PointValueWithCovariancePairType = typename statismo::StatisticalModel<T>::PointValueWithCovariancePairType ;
  using PointValueWithCovarianceListType = typename statismo::StatisticalModel<T>::PointValueWithCovarianceListType ;
  using DomainType  = typename statismo::StatisticalModel<T>::DomainType ;

  StatisticalModel() = default;
  virtual ~StatisticalModel() = default;

  const RepresenterType *
  GetRepresenter() const
  {
    return this->CallForwardImplTrans(statismo::itk::ExceptionHandler{ *this }, &ImplType::GetRepresenter);
  }

  const DomainType &
  GetDomain() const
  {
    return this->CallForwardImplTrans(statismo::itk::ExceptionHandler{ *this }, &ImplType::GetDomain);
  }

  DatasetPointerType
  DrawMean() const
  {
    return this->CallForwardImplTrans(statismo::itk::ExceptionHandler{ *this }, &ImplType::DrawMean);
  }

  ValueType
  DrawMeanAtPoint(const PointType & pt) const
  {
    using OverloadType = ValueType (ImplType::*)(const PointType &) const;
    return this->CallForwardImplTrans(
      statismo::itk::ExceptionHandler{ *this }, static_cast<OverloadType>(&ImplType::DrawMeanAtPoint), pt);
  }

  ValueType
  DrawMeanAtPoint(unsigned ptid) const
  {
    using OverloadType = ValueType (ImplType::*)(unsigned) const;
    return this->CallForwardImplTrans(
      statismo::itk::ExceptionHandler{ *this }, static_cast<OverloadType>(&ImplType::DrawMeanAtPoint), ptid);
  }

  DatasetPointerType
  DrawSample(const VectorType & coeffs, bool addNoise = false) const
  {
    using OverloadType = DatasetPointerType (ImplType::*)(const statismo::VectorType &, bool) const;
    return this->CallForwardImplTrans(
      statismo::itk::ExceptionHandler{ *this }, static_cast<OverloadType>(&ImplType::DrawSample), fromVnlVector(coeffs), addNoise);
  }

  DatasetPointerType
  DrawSample(bool addNoise = false) const
  {
    using OverloadType = DatasetPointerType (ImplType::*)(bool) const;
    return this->CallForwardImplTrans(
      statismo::itk::ExceptionHandler{ *this }, static_cast<OverloadType>(&ImplType::DrawSample), addNoise);
  }

  DatasetPointerType
  DrawPCABasisSample(unsigned componentNumber) const
  {
    using OverloadType = DatasetPointerType (ImplType::*)(unsigned) const;
    return this->CallForwardImplTrans(
      statismo::itk::ExceptionHandler{ *this }, static_cast<OverloadType>(&ImplType::DrawPCABasisSample), componentNumber);
  }

  ValueType
  DrawSampleAtPoint(const VectorType & coeffs, const PointType & pt, bool addNoise = false) const
  {
    using OverloadType = ValueType (ImplType::*)(const statismo::VectorType &, const PointType &, bool) const;
    return this->CallForwardImplTrans(statismo::itk::ExceptionHandler{ *this },
                                      static_cast<OverloadType>(&ImplType::DrawSampleAtPoint),
                                      fromVnlVector(coeffs),
                                      pt,
                                      addNoise);
   }

  ValueType
  DrawSampleAtPoint(const VectorType & coeffs, unsigned ptid, bool addNoise = false) const
  {
    using OverloadType = ValueType (ImplType::*)(const statismo::VectorType &, unsigned, bool) const;
    return this->CallForwardImplTrans(statismo::itk::ExceptionHandler{ *this },
                                      static_cast<OverloadType>(&ImplType::DrawSampleAtPoint),
                                      fromVnlVector(coeffs),
                                      ptid,
                                      addNoise);
  }

  VectorType
  ComputeCoefficients(DatasetConstPointerType ds) const
  {
    return toVnlVector(this->CallForwardImplTrans(statismo::itk::ExceptionHandler{ *this }, &ImplType::ComputeCoefficients, ds));

  }

  double
  ComputeLogProbability(DatasetConstPointerType ds) const
  {
    return this->CallForwardImplTrans(statismo::itk::ExceptionHandler{ *this }, &ImplType::ComputeLogProbability, ds);
  }

  double
  ComputeProbability(DatasetConstPointerType ds) const
  {
    return this->CallForwardImplTrans(statismo::itk::ExceptionHandler{ *this }, &ImplType::ComputeProbability, ds);
  }

  double
  ComputeLogProbabilityOfCoefficients(const VectorType & coeffs) const
  {
    return this->CallForwardImplTrans(
      statismo::itk::ExceptionHandler{ *this }, &ImplType::ComputeLogProbabilityOfCoefficients, fromVnlVector(coeffs));
  }

  double
  ComputeProbabilityOfCoefficients(const VectorType & coeffs) const
  {
    return this->CallForwardImplTrans(
      statismo::itk::ExceptionHandler{ *this }, &ImplType::ComputeProbabilityOfCoefficients, fromVnlVector(coeffs));
  }

  double
  ComputeMahalanobisDistance(DatasetConstPointerType ds) const
  {
    return this->CallForwardImplTrans(statismo::itk::ExceptionHandler{ *this }, &ImplType::ComputeMahalanobisDistance, ds);
  }

  VectorType
  ComputeCoefficientsForPointValues(const PointValueListType & pvlist, double variance) const
  {
    using OverloadType = statismo::VectorType (ImplType::*)(const PointValueListType &, double) const;
    return toVnlVector(
      this->CallForwardImplTrans(statismo::itk::ExceptionHandler{ *this },
                                 static_cast<OverloadType>(&ImplType::ComputeCoefficientsForPointValues),
                                 pvlist,
                                 variance));
 }

  VectorType
  ComputeCoefficientsForPointValuesWithCovariance(const PointValueWithCovarianceListType & pvclist) const
  {
    using OverloadType = statismo::VectorType (ImplType::*)(const PointValueWithCovarianceListType &) const;
    return toVnlVector(
      this->CallForwardImplTrans(statismo::itk::ExceptionHandler{ *this },
                                 static_cast<OverloadType>(&ImplType::ComputeCoefficientsForPointValuesWithCovariance),
                                 pvclist));
  }

  unsigned
  GetNumberOfPrincipalComponents() const
  {
    return this->CallForwardImplTrans(statismo::itk::ExceptionHandler{ *this }, &ImplType::GetNumberOfPrincipalComponents);
 }

  float
  GetNoiseVariance() const
  {
    return this->CallForwardImplTrans(statismo::itk::ExceptionHandler{ *this }, &ImplType::GetNoiseVariance);
  }

  MatrixType
  GetCovarianceAtPoint(const PointType & pt1, const PointType & pt2) const
  {
    using OverloadType = statismo::MatrixType (ImplType::*)(const PointType &, const PointType &) const;
    return toVnlMatrix(this->CallForwardImplTrans(
      statismo::itk::ExceptionHandler{ *this }, static_cast<OverloadType>(&ImplType::GetCovarianceAtPoint), pt1, pt2));
 }

  MatrixType
  GetCovarianceAtPoint(unsigned ptid1, unsigned ptid2) const
  {
    using OverloadType = statismo::MatrixType (ImplType::*)(unsigned, unsigned) const;
    return toVnlMatrix(this->CallForwardImplTrans(
      statismo::itk::ExceptionHandler{ *this }, static_cast<OverloadType>(&ImplType::GetCovarianceAtPoint), ptid1, ptid2));
   }

  MatrixType
  GetJacobian(const PointType & pt) const
  {
    using OverloadType = statismo::MatrixType (ImplType::*)(const PointType &) const;
    return toVnlMatrix(
      this->CallForwardImplTrans(statismo::itk::ExceptionHandler{ *this }, static_cast<OverloadType>(&ImplType::GetJacobian), pt));
  }

  MatrixType
  GetJacobian(unsigned ptId) const
  {
    using OverloadType = statismo::MatrixType (ImplType::*)(unsigned) const;
    return toVnlMatrix(
      this->CallForwardImplTrans(statismo::itk::ExceptionHandler{ *this }, static_cast<OverloadType>(&ImplType::GetJacobian), ptId));
  }

  MatrixType
  GetPCABasisMatrix() const
  {
    return toVnlMatrix(this->CallForwardImplTrans(statismo::itk::ExceptionHandler{ *this }, &ImplType::GetPCABasisMatrix));
  }

  MatrixType
  GetOrthonormalPCABasisMatrix() const
  {
    return toVnlMatrix(this->CallForwardImplTrans(statismo::itk::ExceptionHandler{ *this }, &ImplType::GetOrthonormalPCABasisMatrix));
  }

  VectorType
  GetPCAVarianceVector() const
  {
    return toVnlVector(this->CallForwardImplTrans(statismo::itk::ExceptionHandler{ *this }, &ImplType::GetPCAVarianceVector));
 }

  VectorType
  GetMeanVector() const
  {
    return toVnlVector(this->CallForwardImplTrans(statismo::itk::ExceptionHandler{ *this }, &ImplType::GetMeanVector));
  }

  const statismo::ModelInfo &
  GetModelInfo() const
  {
    return this->CallForwardImplTrans(statismo::itk::ExceptionHandler{ *this }, &ImplType::GetModelInfo);
  }

private:
  static MatrixType
  toVnlMatrix(const statismo::MatrixType & M)
  {
    return MatrixType(M.data(), M.rows(), M.cols());
  }

  static VectorType
  toVnlVector(const statismo::VectorType & v)
  {
    return VectorType(v.data(), v.rows());
  }

  static statismo::VectorType
  fromVnlVector(const VectorType & v)
  {
    return Eigen::Map<const statismo::VectorType>(v.data_block(), v.size());
  }
};


} // namespace itk

#endif
