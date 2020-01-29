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

#ifndef __STATISMO_VTK_STANDARD_IMAGE_REPRESENTER_H_
#define __STATISMO_VTK_STANDARD_IMAGE_REPRESENTER_H_

#include "statismo/core/CommonTypes.h"
#include "statismo/core/Domain.h"
#include "statismo/core/Representer.h"
#include "statismo/VTK/vtkPoint.h"
#include "statismo/VTK/vtkNDPixel.h"

#include <H5Cpp.h>

#include <vtkSmartPointer.h>
#include <vtkStructuredPoints.h>

namespace statismo
{
template <>
struct RepresenterTraits<vtkStructuredPoints>
{
  using DatasetPointerType = vtkSmartPointer<vtkStructuredPoints>;
  using DatasetConstPointerType = const vtkStructuredPoints *;

  using PointType = vtkPoint;
  using ValueType = vtkNDPixel;

  ///@}
};

/**
 * \brief Representer class for vtkStructuredPoints of arbitrary scalar type and PixelDimension
 *
 * The pixel values used for the shape model are stored in the vtkPointData object of the vtkStructuredPoints.
 * They can be either scalars (cf. sp->GetPointData()->GetScalars()) or vectors (cf. sp->GetPointData()->GetVectors()).
 *
 * If you supply vtkStructuredPoints images with several arrays (e.g. scalars, vectors, tensors etc.), you need to
 * ensure that the array relevant to the shape model is the first array, i.e. the one returned by
 * sp->GetPointData()->GetArray(0).
 *
 * \see Representer
 */
template <class TScalar, unsigned PIXEL_DIMENSIONS>
class vtkStandardImageRepresenter
  : public RepresenterBase<vtkStructuredPoints, vtkStandardImageRepresenter<TScalar, PIXEL_DIMENSIONS>>
{
public:
  using RepresenterBaseType =
    RepresenterBase<vtkStructuredPoints, vtkStandardImageRepresenter<TScalar, PIXEL_DIMENSIONS>>;
  friend RepresenterBaseType;
  friend typename RepresenterBaseType::ObjectFactoryType;

  using DatasetPointerType = typename RepresenterBaseType::DatasetPointerType;
  using DatasetConstPointerType = typename RepresenterBaseType::DatasetConstPointerType;
  using PointType = typename RepresenterBaseType::PointType;
  using DomainType = typename RepresenterBaseType::DomainType;
  using ValueType = typename RepresenterBaseType::ValueType;

  void
  Load(const H5::Group & fg) override;
  void
  Save(const H5::Group & fg) const override;

  void DeleteDataset(DatasetPointerType) const override{
    // no op as smart pointers are now use a data type
  };

  DatasetPointerType
  CloneDataset(DatasetConstPointerType d) const override
  {
    auto clone = vtkSmartPointer<vtkStructuredPoints>::New();
    clone->DeepCopy(const_cast<vtkStructuredPoints *>(d));
    return clone;
  }

  const DomainType &
  GetDomain() const override
  {
    return m_domain;
  }

  DatasetConstPointerType
  GetReference() const override
  {
    return m_reference;
  }

  statismo::VectorType
  PointToVector(const PointType & pt) const override;
  statismo::VectorType
  SampleToSampleVector(DatasetConstPointerType sample) const override;
  DatasetPointerType
  SampleVectorToSample(const statismo::VectorType & sampleVector) const override;


  ValueType
  PointSampleFromSample(DatasetConstPointerType sample, unsigned ptid) const override;
  statismo::VectorType
  PointSampleToPointSampleVector(const ValueType & v) const override;
  ValueType
  PointSampleVectorToPointSample(const statismo::VectorType & pointSample) const override;

  unsigned
  GetPointIdForPoint(const PointType & pt) const override;

  unsigned
  GetNumberOfPoints() const;

  static unsigned
  GetNumberOfPoints(DatasetPointerType reference);

private:
  static unsigned
  GetDimensionsImpl()
  {
    return PIXEL_DIMENSIONS;
  }

  static std::string
  GetVersionImpl()
  {
    return "1.0";
  }

  static RepresenterDataType
  GetTypeImpl()
  {
    return RepresenterDataType::IMAGE;
  }

  static std::string
  GetNameImpl()
  {
    return "vtkStandardImageRepresenter";
  }

  void
  AssertCompatibility(DatasetConstPointerType data) const;

  vtkStandardImageRepresenter *
  CloneImpl() const override;

  vtkSmartPointer<vtkStructuredPoints>
  LoadRefLegacy(const H5::Group & fg) const;
  vtkSmartPointer<vtkStructuredPoints>
  LoadRef(const H5::Group & fg) const;

  void
  SetReference(DatasetConstPointerType reference);

  explicit vtkStandardImageRepresenter(DatasetConstPointerType reference);

  vtkStandardImageRepresenter()
    : m_reference(DatasetPointerType::New())
  {}

  DatasetPointerType m_reference;
  DomainType         m_domain;
};

} // namespace statismo

#include "vtkStandardImageRepresenter.hxx"

#endif
