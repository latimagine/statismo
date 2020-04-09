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

#ifndef __STATIMO_ITK_STANDARD_IMAGE_REPRESENTER_H_
#define __STATIMO_ITK_STANDARD_IMAGE_REPRESENTER_H_

#include "statismo/core/CommonTypes.h"
#include "statismo/core/Representer.h"
#include "statismo/ITK/itkPixelConversionTraits.h"
#include "statismo/ITK/itkPointTraits.h"

#include "statismo/ITK/itkConfig.h" // this needs to be the first include

#include <H5Cpp.h>

#include <itkObject.h>
#include <itkImage.h>

namespace statismo
{
/**
 * \brief Representer trait specialization
 * \ingroup Representers
 * \ingroup ITK
 */
template <typename T, auto N>
struct RepresenterTraits<::itk::Image<T, N>>
{
  using VectorImageType = ::itk::Image<T, N>;
  using DatasetPointerType = typename VectorImageType::Pointer;
  using DatasetConstPointerType = typename VectorImageType::Pointer;
  using PointType = typename VectorImageType::PointType;
  using ValueType = typename VectorImageType::PixelType;
};
} // namespace statismo

namespace itk
{

/**
 * \brief Representer for scalar and vector valued images
 * \ingroup Representers
 * \ingroup ITK
 */
template <typename Pixel, unsigned IMAGE_DIMENSION>
class StandardImageRepresenter
  : public Object
  , public statismo::RepresenterBase<itk::Image<Pixel, IMAGE_DIMENSION>,
                                     StandardImageRepresenter<Pixel, IMAGE_DIMENSION>>
{
public:
  using Self = StandardImageRepresenter;
  using Superclass = Object;
  using Pointer = SmartPointer<Self>;
  using ConstPointer = SmartPointer<const Self>;
  using Base =
    statismo::RepresenterBase<itk::Image<Pixel, IMAGE_DIMENSION>, StandardImageRepresenter<Pixel, IMAGE_DIMENSION>>;
  friend Base;
  friend typename Base::ObjectFactoryType;

  /** New macro for creation of through a Smart Pointer */
  itkSimpleNewMacro(Self);

  /** Run-time type information (and related methods) */
  itkTypeMacro(StandardImageRepresenter, Object);

  using ImageType = itk::Image<Pixel, IMAGE_DIMENSION>;
  using RepresenterBaseType = typename statismo::Representer<ImageType>;
  using DomainType = typename RepresenterBaseType::DomainType;
  using PointType = typename RepresenterBaseType::PointType;
  using ValueType = typename RepresenterBaseType::ValueType;
  using DatasetType = typename RepresenterBaseType::DatasetType;
  using DatasetPointerType = typename RepresenterBaseType::DatasetPointerType;
  using DatasetConstPointerType = typename RepresenterBaseType::DatasetConstPointerType;

  StandardImageRepresenter();
  virtual ~StandardImageRepresenter(); // NOLINT

  void
  Delete() override
  {
    this->UnRegister();
  }

  const DomainType &
  GetDomain() const override
  {
    return m_domain;
  }

  void DeleteDataset(DatasetConstPointerType) const override
  {
    // no op as ITK uses ref counted smart pointers
  }
  DatasetPointerType
  CloneDataset(DatasetConstPointerType d) const override;

  void
  Load(const H5::Group & fg) override;
  void
  Save(const H5::Group & fg) const override;

  DatasetConstPointerType
  GetReference() const override
  {
    return m_reference;
  }

  void
  SetReference(ImageType * reference);

  statismo::VectorType
  PointToVector(const PointType & pt) const override;
  statismo::VectorType
  SampleToSampleVector(DatasetConstPointerType sample) const override;
  DatasetPointerType
  SampleVectorToSample(const statismo::VectorType & sample) const override;

  ValueType
  PointSampleFromSample(DatasetConstPointerType sample, unsigned ptid) const override;
  ValueType
  PointSampleVectorToPointSample(const statismo::VectorType & pointSample) const override;
  statismo::VectorType
  PointSampleToPointSampleVector(const ValueType & v) const override;

  unsigned
  GetPointIdForPoint(const PointType & pt) const override;

  virtual unsigned
  GetNumberOfPoints() const;

private:
  static unsigned
  GetDimensionsImpl()
  {
    return PixelConversionTrait<Pixel>::GetPixelDimension();
  }
  static std::string
  GetNameImpl()
  {
    return "itkStandardImageRepresenter";
  }
  static statismo::RepresenterDataType
  GetTypeImpl()
  {
    return statismo::RepresenterDataType::IMAGE;
  }

  static std::string
  GetVersionImpl()
  {
    return "0.1";
  }

  StandardImageRepresenter *
  CloneImpl() const override;

  typename ImageType::Pointer
  LoadRef(const H5::Group & fg) const;
  typename ImageType::Pointer
  LoadRefLegacy(const H5::Group & fg) const;

  DatasetConstPointerType m_reference;
  DomainType              m_domain;
};

} // namespace itk

#include "itkStandardImageRepresenter.hxx"

#endif
