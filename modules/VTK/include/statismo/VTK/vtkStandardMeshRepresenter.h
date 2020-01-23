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


#ifndef __STATISMO_VTK_STANDARD_MESH_REPRESENTER_H_
#define __STATISMO_VTK_STANDARD_MESH_REPRESENTER_H_

#include "statismo/core/CommonTypes.h"
#include "statismo/core/Domain.h"
#include "statismo/core/Representer.h"
#include "statismo/VTK/vtkPoint.h"

#include <H5Cpp.h>

#include <vtkPolyData.h>
#include <vtkSmartPointer.h>

namespace statismo
{

template <>
struct RepresenterTraits<vtkPolyData>
{
  using DatasetPointerType = vtkSmartPointer<vtkPolyData>;
  using DatasetConstPointerType = const vtkPolyData *;
  using PointType = vtkPoint;
  using ValueType = vtkPoint;
};

/**
 * \brief A representer for vtkPolyData, which stores the representer data in the standard
 * mesh format defined for statismo.
 *
 * \see Representer
 */
class vtkStandardMeshRepresenter : public RepresenterBase<vtkPolyData, vtkStandardMeshRepresenter>
{
public:
  using RepresenterBaseType = RepresenterBase<vtkPolyData, vtkStandardMeshRepresenter>;
  friend RepresenterBaseType;
  friend typename RepresenterBaseType::ObjectFactoryType;

  void
  Load(const H5::Group & fg) override;

  void
  Save(const H5::Group & fg) const override;

  const DomainType &
  GetDomain() const override
  {
    return m_domain;
  }

  void DeleteDataset(DatasetPointerType) const override{
    // no op as smart pointers are now use a data type
  };

  DatasetPointerType
  CloneDataset(DatasetConstPointerType d) const override
  {
    auto clone = DatasetPointerType::New();
    clone->DeepCopy(const_cast<vtkPolyData *>(d));
    return clone;
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
  SampleVectorToSample(const statismo::VectorType & sample) const override;

  ValueType
  PointSampleFromSample(DatasetConstPointerType sample, unsigned ptid) const override;
  statismo::VectorType
  PointSampleToPointSampleVector(const ValueType & v) const override;
  ValueType
  PointSampleVectorToPointSample(const statismo::VectorType & v) const override;

  unsigned
  GetNumberOfPoints() const;
  unsigned
  GetPointIdForPoint(const PointType & pt) const override;

private:
  vtkStandardMeshRepresenter()
    : m_reference(DatasetPointerType::New())
  {}
  explicit vtkStandardMeshRepresenter(const std::string & reference);
  explicit vtkStandardMeshRepresenter(DatasetConstPointerType reference);

  vtkStandardMeshRepresenter *
  CloneImpl() const override;

  static std::string
  GetNameImpl()
  {
    return "vtkStandardMeshRepresenter";
  }
  static unsigned
  GetDimensionsImpl()
  {
    return 3;
  }
  static std::string
  GetVersionImpl()
  {
    return "1.0";
  }
  static RepresenterDataType
  GetTypeImpl()
  {
    return RepresenterDataType::POLYGON_MESH;
  }

  void
  SetReference(DatasetConstPointerType reference);

  DatasetPointerType
  LoadRefLegacy(const H5::Group & fg) const;
  DatasetPointerType
  LoadRef(const H5::Group & fg) const;

  void
  WriteDataArray(const H5::H5Location & group, const std::string & name, const vtkDataArray * ds) const;
  static vtkSmartPointer<vtkDataArray>
  GetAsDataArray(const H5::Group & group, const std::string & name);
  static void
  FillDataArray(const statismo::GenericEigenTraits<double>::MatrixType & m, vtkDataArray * dataArray);

  DatasetPointerType m_reference;
  DomainType         m_domain;
};

} // namespace statismo

#endif
