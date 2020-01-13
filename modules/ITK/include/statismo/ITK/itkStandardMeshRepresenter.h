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


#ifndef __STATIMO_ITK_STANDARD_MESH_REPRESENTER_H_
#define __STATIMO_ITK_STANDARD_MESH_REPRESENTER_H_

#include "statismo/core/CommonTypes.h"
#include "statismo/core/Exceptions.h"
#include "statismo/core/Representer.h"
#include "statismo/core/Hash.h"
#include "statismo/ITK/itkConfig.h" // this needs to be the first include file
#include "statismo/ITK/itkPixelConversionTraits.h"

#include <itkMesh.h>
#include <itkObject.h>

#include <unordered_map>

namespace statismo
{
template <typename T, auto N>
struct RepresenterTraits<::itk::Mesh<T, N>>
{
  using MeshType = ::itk::Mesh<T, N>;
  using DatasetPointerType = typename MeshType::Pointer;
  using DatasetConstPointerType = typename MeshType::Pointer;
  using PointType = typename MeshType::PointType;
  using ValueType = typename MeshType::PointType;

  static constexpr unsigned Dimension = N;
};

} // namespace statismo

namespace itk
{

/**
 * \ingroup Representers
 * \brief A representer for scalar valued itk Meshs
 * \sa Representer
 */
template <class TPixel, unsigned MeshDimension>
class StandardMeshRepresenter
  : public statismo::RepresenterBase<itk::Mesh<TPixel, MeshDimension>, StandardMeshRepresenter<TPixel, MeshDimension>>
  , public Object
{
public:
  /* Standard class typedefs. */
  using Self = StandardMeshRepresenter;
  using Superclass = Object;
  using Pointer = SmartPointer<Self>;
  using ConstPointer = SmartPointer<const Self>;
  using Base =
    statismo::RepresenterBase<itk::Mesh<TPixel, MeshDimension>, StandardMeshRepresenter<TPixel, MeshDimension>>;
  friend Base;
  friend typename Base::ObjectFactoryType;

  using MeshType = itk::Mesh<TPixel, MeshDimension>;
  using RepresenterBaseType = typename statismo::Representer<MeshType>;
  using DomainType = typename RepresenterBaseType::DomainType;
  using PointType = typename RepresenterBaseType::PointType;
  using ValueType = typename RepresenterBaseType::ValueType;
  using DatasetPointerType = typename RepresenterBaseType::DatasetPointerType;
  using DatasetConstPointerType = typename RepresenterBaseType::DatasetConstPointerType;
  using PointsContainerType = typename MeshType::PointsContainer;

  /// The type of the data set to be used
  using DatasetType = MeshType;

  // An unordered map is used to cache pointid for corresponding points
  using PointCacheType = std::unordered_map<PointType, unsigned, statismo::Hash<PointType>>;

  /** New macro for creation of through a Smart Pointer. */
  itkSimpleNewMacro(Self);

  /** Run-time type information (and related methods). */
  itkTypeMacro(StandardMeshRepresenter, Object);

  StandardMeshRepresenter();
  virtual ~StandardMeshRepresenter();

  virtual void
  Delete() override
  {
    this->UnRegister();
  }

  virtual void DeleteDataset(DatasetPointerType) const override{
    // no op
  };
  virtual DatasetPointerType
  CloneDataset(DatasetConstPointerType mesh) const override;

  virtual void
  Load(const H5::Group & fg) override;

  virtual void
  Save(const H5::Group & fg) const override;

  virtual const DomainType &
  GetDomain() const override
  {
    return m_domain;
  }

  virtual void
  SetReference(DatasetPointerType ds);

  virtual DatasetConstPointerType
  GetReference() const override
  {
    return m_reference;
  }

  virtual statismo::VectorType
  PointToVector(const PointType & pt) const override;

  virtual statismo::VectorType
  SampleToSampleVector(DatasetConstPointerType sample) const override;

  virtual DatasetPointerType
  SampleVectorToSample(const statismo::VectorType & sample) const override;

  virtual ValueType
  PointSampleFromSample(DatasetConstPointerType sample, unsigned ptid) const override;

  virtual ValueType
  PointSampleVectorToPointSample(const statismo::VectorType & pointSample) const override;

  virtual statismo::VectorType
  PointSampleToPointSampleVector(const ValueType & v) const override;

  virtual unsigned
  GetNumberOfPoints() const;

  /// return the point id associated with the given point
  /// \warning This works currently only for points that are defined on the reference
  virtual unsigned
  GetPointIdForPoint(const PointType & point) const override;

private:
  static unsigned
  GetDimensionsImpl()
  {
    return MeshDimension;
  }
  static std::string
  GetNameImpl()
  {
    return "itkStandardMeshRepresenter";
  }
  static statismo::RepresenterDataType
  GetTypeImpl()
  {
    return statismo::RepresenterDataType::POLYGON_MESH;
  }
  static std::string
  GetVersionImpl()
  {
    return "0.1";
  }

  StandardMeshRepresenter *
  CloneImpl() const override;

  typename MeshType::Pointer
  LoadRef(const H5::Group & fg) const;
  typename MeshType::Pointer
  LoadRefLegacy(const H5::Group & fg) const;

  // returns the closest point for the given mesh
  unsigned
  FindClosestPoint(const MeshType * mesh, const PointType pt) const;

  DatasetConstPointerType m_reference;
  DomainType              m_domain;
  mutable PointCacheType  m_pointCache;
};


} // namespace itk


#include "itkStandardMeshRepresenter.hxx"

#endif
