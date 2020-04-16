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
#ifndef __STATIMO_CORE_REPRESENTER_H_
#define __STATIMO_CORE_REPRESENTER_H_

#include "statismo/core/CommonTypes.h"
#include "statismo/core/Domain.h"
#include "statismo/core/CoreTraits.h"
#include "statismo/core/Clonable.h"
#include "statismo/core/GenericFactory.h"
#include "statismo/core/Logger.h"

#include <H5Cpp.h>
#include <string>
#include <memory>

/**
 * \defgroup Representers Representers classes and routines
 */

namespace statismo
{

/**
 * \brief Traits class that must be defined for new representers
 *
 * When defining a new representer, a traits class must be bound to it with the
 * following attributes:
 * - DatasetPointerType
 * - DatasetConstPointerType
 * - PointType
 * - ValueType
 *
 * \ingroup Representers
 * \ingroup Core
 */
template <class T>
struct RepresenterTraits;

/**
 * \brief Data type manipulated by the representer
 * \ingroup Representers
 */
enum class RepresenterDataType
{
  UNKNOWN = 0,
  POINT_SET = 1,
  POLYGON_MESH = 2,
  VOLUME_MESH = 3,
  IMAGE = 4,
  VECTOR = 5,
  CUSTOM = 99
};

/**
 * \brief Provides the interface between statismo framework and the dataset type an application
 * layer uses.
 *
 * A Representer is a type that provides the connection between statismo library
 * and the application layer. It distinguishes three different representations of the data, and provides methods for
 * conversion between those representations:
 * - a Dataset, typically as read from a file on the disk
 * - a Sample, which is a geometric (generally a rigid or affine) transform of the dataset
 * - a SampleVector, which is an internal representation (vector) useful from the statistical analysis.
 *
 * A good starting point to define a new representer is to look at \a TrivialVectorialRepresenter
 *
 * \ingroup Representers
 * \ingroup Core
 */
template <class T>
class Representer : public Clonable<Representer<T>>
{

public:
  static RepresenterDataType
  TypeFromString(const std::string & s)
  {
    if (s == "POINT_SET")
    {
      return RepresenterDataType::POINT_SET;
    }

    if (s == "POLYGON_MESH")
    {
      return RepresenterDataType::POLYGON_MESH;
    }

    if (s == "VOLUME_MESH")
    {
      return RepresenterDataType::VOLUME_MESH;
    }

    if (s == "IMAGE")
    {
      return RepresenterDataType::IMAGE;
    }

    if (s == "VECTOR")
    {
      return RepresenterDataType::VECTOR;
    }

    if (s == "CUSTOM")
    {
      return RepresenterDataType::CUSTOM;
    }

    return RepresenterDataType::UNKNOWN;
  }

  static std::string
  TypeToString(RepresenterDataType type)
  {
    switch (type)
    {
      case RepresenterDataType::POINT_SET:
        return "POINT_SET";
      case RepresenterDataType::POLYGON_MESH:
        return "POLYGON_MESH";
      case RepresenterDataType::VOLUME_MESH:
        return "VOLUME_MESH";
      case RepresenterDataType::IMAGE:
        return "IMAGE";
      case RepresenterDataType::VECTOR:
        return "VECTOR";
      case RepresenterDataType::CUSTOM:
        return "CUSTOM";
      default:
        break;
    }

    return "UNKNOWN";
  }
  /**
   * \name Type definitions
   */
  ///@{
  /// Defines (a pointer to) the type of the dataset that is represented.
  /// This could either be a naked pointer or a smart pointer.
  using DatasetPointerType = typename RepresenterTraits<T>::DatasetPointerType;

  /// Defines the const pointer type o fthe datset that is represented
  using DatasetConstPointerType = typename RepresenterTraits<T>::DatasetConstPointerType;

  /// Defines the pointtype of the dataset
  using PointType = typename RepresenterTraits<T>::PointType;

  /// Defines the type of the value when the dataset is evaluated at a given point
  /// (for a image, this could for example be a scalar value or an RGB value)
  using ValueType = typename RepresenterTraits<T>::ValueType;

  using DatasetType = T;
  using DomainType = Domain<PointType>;

  ///
  /// Defines the real (computational) dimension of the point type
  ///
  /// Even if the data dimension given by \a GetDimensions is 2 (for instance, a 2D image),
  /// the points used to store data could be a 3 dimension vector (with last dimension unused).
  /// In this case, the real dimension (used for computation) is 3.
  ///
  static constexpr unsigned sk_realPointDimension = PointTraits<PointType>::sk_realDimension;

  ///@}

  /**
   * \name Representer attributes
   */
  ///@{

  /**
   * \brief Get representer identifier
   */
  virtual std::string
  GetName() const = 0;

  /**
   * \brief Get representer data type
   */
  virtual RepresenterDataType
  GetType() const = 0;

  /**
   * \brief Get representer version
   */
  virtual std::string
  GetVersion() const = 0;

  /**
   * \brief Get the dimensionality of the dataset
   * - for a mesh, should be 3
   * - for a scalar image, should be 1
   * - etc.
   */
  virtual unsigned
  GetDimensions() const = 0;

  ///@}


  /**
   * \name Object creation and destruction
   */
  ///@{
  /**
   * \brief Creates a new representer object, with the
   * the information defined in the given hdf5 group
   * \sa Save
   */
  virtual void
  Load(const H5::Group & fg) = 0;

  /**
   * \brief Delete the representer object
   */
  virtual void
  Delete() = 0;

  /**
   * \brief Get logger
   */
  virtual Logger *
  GetLogger() const = 0;

  /**
   * \brief Delete a dataset
   */
  virtual void
  DeleteDataset(DatasetPointerType d) const = 0;

  /**
   * \brief Clone a dataset
   */
  virtual DatasetPointerType
  CloneDataset(DatasetConstPointerType d) const = 0;
  ///@}

  /**
   * \name Conversion from the dataset to a vector representation and back
   */
  ///@{

  /**
   * \brief Get the Domain for this representer
   *
   * The domain is essentially a list of all the points on which the model is
   * defined.
   */
  virtual const statismo::Domain<PointType> &
  GetDomain() const = 0;

  /**
   * \brief Get reference dataset
   */
  virtual DatasetConstPointerType
  GetReference() const = 0;

  /**
   * \brief Convert a dataset point to an internal vector
   */
  virtual VectorType
  PointToVector(const PointType & pt) const = 0;

  /**
   * \brief Get a vectorial representation of a sample
   */
  virtual VectorType
  SampleToSampleVector(DatasetConstPointerType sample) const = 0;

  /**
   * \brief Get a sample from an internal vector
   */
  virtual DatasetPointerType
  SampleVectorToSample(const VectorType & sample) const = 0;

  /**
   * \brief Get a \a sample value at point with index \a ptid
   */
  virtual ValueType
  PointSampleFromSample(DatasetConstPointerType sample, unsigned ptid) const = 0;

  /**
   * \brief Take a point sample (i.e. the value of a sample at a given point) and converts it
   * to its vector representation
   *
   * The type of the point sample is a ValueType, that depends on the type of the dataset.
   * For a mesh this would for example be a 3D point,
   * while for a scalar image this would be a scalar value representing the intensity.
   */
  virtual ValueType
  PointSampleVectorToPointSample(const VectorType & v) const = 0;

  /**
   * \brief Convert the given vector representation of a pointSample back to its ValueType
   * \sa PointSampleVectorToPointSample
   */
  virtual VectorType
  PointSampleToPointSampleVector(const ValueType & pointSample) const = 0;

  /**
   * \brief Define the mapping between the point ids and the position in the vector.
   *
   * Assuming for example that a 3D mesh type is represented.
   * A conversion strategy used in DatasetToSampleVector could be to return
   * a vector \f$(pt1_x, pt1_y, pt1_z, ..., ptn_x, ptn_y, ptn_z\f$.
   * In this case, this method would return for inputs \a ptId, \a componentId
   * the value \a ptId * 3 + \a componentId
   */
  virtual unsigned
  MapPointIdToInternalIdx(unsigned ptId, unsigned componentInd) const = 0;

  /**
   * \brief Given a point (the coordinates) return the pointId of this point
   */
  virtual unsigned
  GetPointIdForPoint(const PointType & point) const = 0;

  ///@}

  /**
   * \name Persistence
   */
  ///@{
  /**
   * \brief Save the informatino that define this representer to the given hdf5 group
   */
  virtual void
  Save(const H5::Group & fg) const = 0;

  ///@}

  /**
   * \name Utilities
   */
  /*
   * \brief Returns a new dataset that corresponds to the zero element of the underlying vectorspace
   * obtained when vectorizing a dataset.
   *
   */
  virtual DatasetPointerType
  IdentitySample() const = 0;
};

/**
 * \brief Base implementation for representers
 *
 * \ingroup Representers
 * \ingroup Core
 */
template <typename T, typename Derived>
class RepresenterBase
  : public Representer<T>
  , public GenericFactory<Derived>
{

public:
  using DatasetPointerType = typename RepresenterTraits<T>::DatasetPointerType;
  using ObjectFactoryType = GenericFactory<Derived>;

  virtual void
  SetLogger(Logger * logger)
  {
    m_logger = logger;
  }

  void
  Delete() override
  {
    delete this;
  }

  std::string
  GetName() const final
  {
    return Derived::GetNameImpl();
  }

  RepresenterDataType
  GetType() const final
  {
    return Derived::GetTypeImpl();
  }

  unsigned
  GetDimensions() const final
  {
    return Derived::GetDimensionsImpl();
  }

  std::string
  GetVersion() const final
  {
    return Derived::GetVersionImpl();
  }

  unsigned
  MapPointIdToInternalIdx(unsigned ptId, unsigned componentInd) const override
  {
    return ptId * GetDimensions() + componentInd;
  }

  DatasetPointerType
  IdentitySample() const override
  {

    switch (this->GetType())
    {
      case RepresenterDataType::POINT_SET:
      case RepresenterDataType::POLYGON_MESH:
      case RepresenterDataType::VOLUME_MESH:
      {
        return this->CloneDataset(this->GetReference());
        break;
      }
      case RepresenterDataType::IMAGE:
      case RepresenterDataType::VECTOR:
      {
        VectorType zeroVec =
          VectorType::Zero(static_cast<uint64_t>(this->GetDomain().GetNumberOfPoints()) * GetDimensions());
        return this->SampleVectorToSample(zeroVec);
        break;
      }
      default:
      {
        throw statismo::StatisticalModelException(
          "No cannonical identityDataset method is defined for custom Representers.");
      }
    }
  }

protected:
  Logger *
  GetLogger() const override
  {
    return m_logger;
  }

  RepresenterBase() = default;

private:
  Logger * m_logger{ nullptr };
};
} // namespace statismo

#endif
