/*
 * DataItem.h
 *
 * Created by Marcel Luethi
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

#ifndef __STATIMO_CORE_DATA_ITEM_H_
#define __STATIMO_CORE_DATA_ITEM_H_

#include "statismo/core/CommonTypes.h"
#include "statismo/core/HDF5Utils.h"
#include "statismo/core/Representer.h"
#include "statismo/core/NonCopyable.h"

#include <vector>

namespace statismo
{

/**
 * \brief Base abstract class for data item
 *
 * A data item holds information about a given sample.
 *
 * \ingroup DataManagers
 * \ingroup Core
 */
template <typename T>
class DataItem : public NonCopyable
{
public:
  using RepresenterType = Representer<T>;
  using DatasetPointerType = typename RepresenterType::DatasetPointerType;

  /**
   * \brief Load sample from group
   */
  virtual void
  LoadFromGroup(const H5::Group & dsGroup) = 0;

  /**
   * \brief Save sample to group
   */
  virtual void
  Save(const H5::Group & dsGroup) const = 0;

  /**
   * \brief Get URI of the original dataset
   */
  virtual std::string
  GetDatasetURI() const = 0;

  /**
   * \brief Get representer used for sample creation
   */
  virtual const RepresenterType *
  GetRepresenter() const = 0;

  /**
   * \brief Get sample vectorial representation
   */
  virtual VectorType
  GetSampleVector() const = 0;

  /**
   * \brief Get the sample
   * \return Sample in the representer representation
   * \warning If returned type is a raw pointer, ownership is transferred
   * to the caller
   */
  virtual DatasetPointerType
  GetSample() const = 0;

  /**
   * \brief Generic delete function
   */
  virtual void
  Delete() = 0;
};

/**
 * \brief Base implementation for data item
 *
 * \ingroup DataManagers
 * \ingroup Core
 */
template <typename T, typename Derived>
class DataItemBase
  : public DataItem<T>
  , public GenericFactory<Derived>
{
public:
  using RepresenterType = typename DataItem<T>::RepresenterType;
  using DatasetPointerType = typename DataItem<T>::DatasetPointerType;
  using ObjectFactoryType = GenericFactory<Derived>;
  friend ObjectFactoryType;

  void
  Save(const H5::Group & dsGroup) const override;

  std::string
  GetDatasetURI() const override
  {
    return m_uri;
  }

  const RepresenterType *
  GetRepresenter() const override
  {
    return m_representer;
  }

  VectorType
  GetSampleVector() const override
  {
    return m_sampleVector;
  }

  DatasetPointerType
  GetSample() const override
  {
    return m_representer->SampleVectorToSample(m_sampleVector);
  }

  void
  Delete() override
  {
    delete this;
  }

protected:
  DataItemBase(const RepresenterType * representer, std::string uri, VectorType sampleVector)
    : m_representer(representer)
    , m_uri(std::move(uri))
    , m_sampleVector(std::move(sampleVector))
  {}

  explicit DataItemBase(const RepresenterType * representer)
    : m_representer(representer)
  {}

  // loads the internal state from the hdf5 file
  void
  LoadFromGroup(const H5::Group & dsGroup) override
  {
    HDF5Utils::ReadVector(dsGroup, "./samplevector", m_sampleVector);
    m_uri = HDF5Utils::ReadString(dsGroup, "./URI");

    LoadInternalImpl(dsGroup);
  }

  void
  SaveToGroup(const H5::Group & dsGroup) const
  {
    HDF5Utils::WriteVector(dsGroup, "./samplevector", m_sampleVector);
    HDF5Utils::WriteString(dsGroup, "./URI", m_uri);

    SaveInternalImpl(dsGroup);
  }

private:
  virtual void
  SaveInternalImpl(const H5::Group & dsGroup) const = 0;

  virtual void
  LoadInternalImpl(const H5::Group & dsGroup) = 0;

  const RepresenterType * m_representer;
  std::string             m_uri;
  VectorType              m_sampleVector;
};

/**
 * \brief Standard data item
 *
 * \ingroup DataManagers
 * \ingroup Core
 */
template <typename T>
class BasicDataItem : public DataItemBase<T, BasicDataItem<T>>
{
public:
  using Superclass = DataItemBase<T, BasicDataItem<T>>;
  using RepresenterType = typename Superclass::RepresenterType;
  friend typename Superclass::ObjectFactoryType;

  /**
   * \brief Load a new data item object
   * \param representer representer
   * \param dsGroup group in the hdf5 file for this dataset
   */
  static UniquePtrType<DataItem<T>>
  Load(const RepresenterType * representer, const H5::Group & dsGroup);

protected:
  void
  SaveInternalImpl(const H5::Group & dsGroup) const override
  {
    HDF5Utils::WriteString(dsGroup, "./sampletype", "DataItem");
  }

  void
  LoadInternalImpl(const H5::Group &) override
  {
    // no op
  }

private:
  BasicDataItem(const RepresenterType * representer, std::string uri, VectorType sampleVector)
    : Superclass(representer, std::move(uri), std::move(sampleVector))
  {}

  explicit BasicDataItem(const RepresenterType * representer)
    : Superclass(representer)
  {}
};

/**
 * \brief Data item with surrogates
 *
 * Specific data item implementation that associates surrogates to the data.
 *
 * This kind of item makes it possible to associate categorical or continuous variables
 * with a sample, in a vectorial representation.
 *
 * The vector is provided by a file providing the values in ascii format (empty space or EOL separating the values)
 * as described in \ref md_data_README "data description".
 *
 * \sa DataItem
 * \sa DataManagerWithSurrogates
 *
 * \ingroup DataManagers
 * \ingroup Core
 */
template <typename T>
class DataItemWithSurrogates : public DataItemBase<T, DataItemWithSurrogates<T>>
{
public:
  enum class SurrogateType
  {
    CATEGORICAL = 0, // e.g. Gender
    CONTINUOUS = 1   // e.g. Size, weight
  };
  using SurrogateTypeVectorType = std::vector<SurrogateType>;
  using Superclass = DataItemBase<T, DataItemWithSurrogates<T>>;
  using RepresenterType = typename Superclass::RepresenterType;
  friend typename Superclass::ObjectFactoryType;

  /**
   * \brief Load a new DataItemWithSurrogates object
   * \param representer representer
   * \param dsGroup group in the hdf5 file for this dataset
   */
  static UniquePtrType<DataItem<T>>
  Load(const RepresenterType * representer, const H5::Group & dsGroup);

  VectorType
  GetSurrogateVector() const
  {
    return m_surrogateVector;
  }

  std::string
  GetSurrogateFilename() const
  {
    return m_surrogateFilename;
  }

private:
  DataItemWithSurrogates(const RepresenterType * representer,
                         std::string             datasetURI,
                         VectorType              sampleVector,
                         std::string             surrogateFilename,
                         VectorType              surrogateVector)
    : Superclass(representer, std::move(datasetURI), std::move(sampleVector))
    , m_surrogateFilename(std::move(surrogateFilename))
    , m_surrogateVector(std::move(surrogateVector))
  {}

  explicit DataItemWithSurrogates(const RepresenterType * representer)
    : Superclass(representer)
  {}

  // loads the internal state from the hdf5 file
  void
  LoadInternalImpl(const H5::Group & dsGroup) override
  {
    HDF5Utils::ReadVector(dsGroup, "./surrogateVector", this->m_surrogateVector);
    m_surrogateFilename = HDF5Utils::ReadString(dsGroup, "./surrogateFilename");
  }

  void
  SaveInternalImpl(const H5::Group & dsGroup) const override
  {
    HDF5Utils::WriteString(dsGroup, "./sampletype", "DataItemWithSurrogates");
    HDF5Utils::WriteVector(dsGroup, "./surrogateVector", this->m_surrogateVector);
    HDF5Utils::WriteString(dsGroup, "./surrogateFilename", this->m_surrogateFilename);
  }

  std::string m_surrogateFilename;
  VectorType  m_surrogateVector;
};


} // namespace statismo

#include "DataItem.hxx"

#endif
