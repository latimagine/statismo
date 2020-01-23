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


#ifndef __STATISMO_VTK_UNSTRUCTURED_GRID_REPRESENTER_H_
#define __STATISMO_VTK_UNSTRUCTURED_GRID_REPRESENTER_H_

#include "statismo/core/CommonTypes.h"
#include "statismo/core/Domain.h"
#include "statismo/core/Representer.h"
#include "statismo/VTK/vtkPoint.h"

#include <H5Cpp.h>

#include <vtkLandmarkTransform.h>
#include <vtkSmartPointer.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkUnstructuredGrid.h>

namespace statismo
{

template <>
struct RepresenterTraits<vtkUnstructuredGrid>
{
  using DatasetPointerType = vtkSmartPointer<vtkUnstructuredGrid>;
  using DatasetConstPointerType = const vtkUnstructuredGrid *;
  using PointType = vtkPoint;
  using ValueType = vtkPoint;
};

/**
 * \brief A representer for vtkUnstructuredGrid using Procrustes alignment to align the datasets
 *
 * This class provides a specialization of the Representer for the type vtkUnstructuredGrid.
 * Procrustes is used to align the given datasets with the reference.
 * The user can choose between Rigid, Similarity and Affine alignment.
 *
 * \note In order to use GPA alignment, simply set the Procrustes Mean as the reference.
 * \warning This class does currently not provide any registration, which implies
 * that the dataset that are read by the class need to be aligned.
 *
 * \see Representer
 */
class vtkUnstructuredGridRepresenter : public RepresenterBase<vtkUnstructuredGrid, vtkUnstructuredGridRepresenter>
{
public:
  using RepresenterBaseType = RepresenterBase<vtkUnstructuredGrid, vtkUnstructuredGridRepresenter>;
  friend RepresenterBaseType;
  friend typename RepresenterBaseType::ObjectFactoryType;

  enum class AlignmentType
  {
    NONE = 999, // something that VTK does not define
    RIGID = VTK_LANDMARK_RIGIDBODY,
    SIMILARITY = VTK_LANDMARK_SIMILARITY,
    AFFINE = VTK_LANDMARK_AFFINE
  };

  void
  Load(const H5::Group & fg) override;

  void
  Save(const H5::Group & fg) const override;

  AlignmentType
  GetAlignment() const
  {
    return m_alignment;
  }

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
    clone->DeepCopy(const_cast<vtkUnstructuredGrid *>(d));
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
  SampleVectorToSample(const statismo::VectorType & sampleVec) const override;

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
  vtkUnstructuredGridRepresenter();
  vtkUnstructuredGridRepresenter(const std::string & reference, AlignmentType alignment);
  vtkUnstructuredGridRepresenter(DatasetConstPointerType reference, AlignmentType alignment);

  static DatasetPointerType
  ReadDataset(const std::string & filename);
  static void
  WriteDataset(const std::string & filename, DatasetConstPointerType pd);

  vtkUnstructuredGridRepresenter *
  CloneImpl() const override;

  static std::string
  GetNameImpl()
  {
    return "vtkUnstructuredGridRepresenter";
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
    return RepresenterDataType::POINT_SET;
  }

  void
  SetReference(DatasetConstPointerType reference);

  DatasetPointerType                          m_reference;
  vtkSmartPointer<vtkTransformPolyDataFilter> m_pdTransform;
  AlignmentType                               m_alignment;
  DomainType                                  m_domain;
};

} // namespace statismo

#endif
