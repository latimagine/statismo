/*
 * DataManagerWithSurrogates.h
 *
 * Created by Marcel Luethi and Remi Blanc
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

#ifndef __STATIMO_CORE_DATA_MANAGER_WITH_SURROGATES_H_
#define __STATIMO_CORE_DATA_MANAGER_WITH_SURROGATES_H_

#include "statismo/core/DataManager.h"

namespace statismo
{

/**
 * \brief Data manager with surrogates
 *
 * Manages data together with surrogate information. The surrogate variables are provided through a vector,
 * and can contain both continuous or categorical data.
 *
 * The surrogate data is provided through files. One file for each dataset, and one file describing
 * the types of surrogates as described in \ref md_data_README "data description"
 *
 * \ingroup DataManagers
 * \ingroup Core
 */
template <typename T>
class DataManagerWithSurrogates : public DataManagerBase<T, DataManagerWithSurrogates<T>>
{

public:
  using Superclass = DataManagerBase<T, DataManagerWithSurrogates<T>>;
  using DataItemType = typename Superclass::DataItemType;
  using RepresenterType = typename Superclass::RepresenterType;
  using DatasetPointerType = typename Superclass::DatasetPointerType;
  using DatasetConstPointerType = typename Superclass::DatasetConstPointerType;
  using BasicDataItemType = BasicDataItem<T>;
  using DataItemWithSurrogatesType = DataItemWithSurrogates<T>;
  using SurrogateTypeVectorType = typename DataItemWithSurrogatesType::SurrogateTypeVectorType;
  using ObjectFactoryType = typename Superclass::ObjectFactoryType;

  friend ObjectFactoryType;

  struct SurrogateTypeInfoType
  {
    SurrogateTypeVectorType types;
    std::string             typeFilename;
  };

  /**
   * \brief Load a new dataManager
   */
  static UniquePtrType<Superclass>
  Load(RepresenterType * representer, const std::string & h5Filename, const std::string & surrogateFilename)
  {
    return Superclass::template Load<DataItemWithSurrogatesType>(representer, h5Filename, surrogateFilename);
  }

  void
  AddDataset(DatasetConstPointerType dataset, const std::string & uri) override;

  /**
   * \brief Add a dataset with surrogate information
   * \param ds dataset filename
   * \param datasetURI URI for the dataset (metadata information)
   * \param surrogateFilename surrogate filename
   */
  void
  AddDatasetWithSurrogates(DatasetConstPointerType ds,
                           const std::string &     datasetURI,
                           const std::string &     surrogateFilename);

  /**
   * \brief Get a vector indicating the types of surrogates variables (Categorical vs Continuous)
   */
  SurrogateTypeVectorType
  GetSurrogateTypes() const
  {
    return m_typeInfo.types;
  }

  /**
   * \brief Get the filename of the file defining the surrogate types
   */
  std::string
  GetSurrogateTypeFilename() const
  {
    return m_typeInfo.typeFilename;
  }

  /**
   * \brief Get a structure containing the type info: vector of types and source filename
   */
  SurrogateTypeInfoType
  GetSurrogateTypeInfo() const
  {
    return m_typeInfo;
  }

protected:
  /**
   * \brief Load information concerning the types of the surrogates variables (categorical=0, continuous=1)
   * \warning it is assumed to be in a text file with the entries separated by spaces or EOL character
   * as described in \ref md_data_README "data description"
   */
  void
  LoadSurrogateTypes(const std::string & filename);

private:
  DataManagerWithSurrogates(const RepresenterType * r, const std::string & filename);

  SurrogateTypeInfoType m_typeInfo;
};

} // namespace statismo

#include "DataManagerWithSurrogates.hxx"

#endif
