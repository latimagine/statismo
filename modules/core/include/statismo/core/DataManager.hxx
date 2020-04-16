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

#ifndef __STATIMO_CORE_DATA_MANAGER_HXX_
#define __STATIMO_CORE_DATA_MANAGER_HXX_

#include "statismo/core/DataManager.h"
#include "statismo/core/ModelInfo.h"
#include "statismo/core/Exceptions.h"
#include "statismo/core/Utils.h"
#include "statismo/core/HDF5Utils.h"
#include "statismo/core/Logger.h"

#include <random>
#include <limits>

namespace statismo
{

template <typename T, typename Derived>
DataManagerBase<T, Derived>::DataManagerBase(const RepresenterType * representer)
  : m_representer{ representer->SafeCloneSelf() }
{
  this->SetLogger(m_representer->GetLogger());
}

template <typename T, typename Derived>
template <typename ConcreteDataItemType, typename... Args>
UniquePtrType<DataManagerBase<T, Derived>>
DataManagerBase<T, Derived>::Load(RepresenterType * representer, const std::string & filename, Args &&... args)
{
  using namespace H5;

  H5File file;
  try
  {
    file = H5File(filename.c_str(), H5F_ACC_RDONLY);
  }
  catch (const H5::Exception & e)
  {
    std::string msg(std::string("could not open HDF5 file \n") + e.getCDetailMsg());
    throw StatisticalModelException(msg.c_str(), Status::IO_ERROR);
  }

  UniquePtrType<DataManagerBase> newDataManagerBase;
  try
  {
    // loading representer
    auto representerGroup = file.openGroup("./representer");
    auto repName = HDF5Utils::ReadStringAttribute(representerGroup, "name");
    auto repTypeStr = HDF5Utils::ReadStringAttribute(representerGroup, "datasetType");
    auto versionStr = HDF5Utils::ReadStringAttribute(representerGroup, "version");
    auto type = RepresenterType::TypeFromString(repTypeStr);
    if (type == RepresenterDataType::CUSTOM || type == RepresenterDataType::UNKNOWN)
    {
      if (repName != representer->GetName())
      {
        std::ostringstream os;
        os << "A different representer was used to create the file and the representer is not of a standard type ";
        os << ("(RepresenterName = ") << repName << " does not match required name = " << representer->GetName() << ")";
        os << "Cannot load hdf5 file";
        throw StatisticalModelException(os.str().c_str(), Status::INVALID_DATA_ERROR);
      }

      if (versionStr != representer->GetVersion())
      {
        std::ostringstream os;
        os << "The version of the representers do not match ";
        os << ("(Version = ") << versionStr << " != = " << representer->GetVersion() << ")";
        os << "Cannot load hdf5 file";
      }
    }

    if (type != representer->GetType())
    {
      std::ostringstream os;
      os << "The representer that was provided cannot be used to load the dataset ";
      os << "(" << RepresenterType::TypeToString(type)
         << " != " << RepresenterType::TypeToString(representer->GetType()) << ").";
      os << "Cannot load hdf5 file.";
      throw StatisticalModelException(os.str().c_str(), Status::INVALID_DATA_ERROR);
    }

    representer->Load(representerGroup);
    newDataManagerBase =
      UniquePtrType<DataManagerBase>(DataManagerBase::Create(representer, std::forward<Args>(args)...));

    auto     dataGroup = file.openGroup("/data");
    unsigned numds = HDF5Utils::ReadInt(dataGroup, "./NumberOfDatasets");
    for (unsigned num = 0; num < numds; num++)
    {
      std::ostringstream ss;
      ss << "./dataset-" << num;

      auto dsGroup = file.openGroup(ss.str().c_str());
      newDataManagerBase->m_dataItemList.push_back(ConcreteDataItemType::Load(representer, dsGroup));
    }
  }
  catch (const H5::Exception & e)
  {
    std::string msg =
      std::string("an exception occurred while reading data matrix to HDF5 file \n") + e.getCDetailMsg();
    throw StatisticalModelException(msg.c_str(), Status::IO_ERROR);
  }

  assert(newDataManagerBase);
  return newDataManagerBase;
}

template <typename T, typename Derived>
void
DataManagerBase<T, Derived>::Save(const std::string & filename) const
{
  STATISMO_LOG_INFO("Saving data to file " + filename);

  using namespace H5;

  assert(m_representer);

  H5File file;
  try
  {
    file = H5File(filename.c_str(), H5F_ACC_TRUNC);
  }
  catch (const H5::Exception & e)
  {
    std::string msg(std::string("Could not open HDF5 file for writing \n") + e.getCDetailMsg());
    STATISMO_LOG_ERROR(msg);
    throw StatisticalModelException(msg.c_str(), Status::IO_ERROR);
  }

  try
  {
    auto representerGroup = file.createGroup("./representer");
    auto dataTypeStr = RepresenterType::TypeToString(m_representer->GetType());

    HDF5Utils::WriteStringAttribute(representerGroup, "name", m_representer->GetName());
    HDF5Utils::WriteStringAttribute(representerGroup, "version", m_representer->GetVersion());
    HDF5Utils::WriteStringAttribute(representerGroup, "datasetType", dataTypeStr);

    this->m_representer->Save(representerGroup);

    auto dataGroup = file.createGroup("./data");

    if (this->m_dataItemList.size() > std::numeric_limits<int>::max())
    {
      STATISMO_LOG_ERROR("Too many dataset to write");
      throw StatisticalModelException("too many dataset to write", Status::OUT_OF_RANGE_ERROR);
    }

    HDF5Utils::WriteInt(dataGroup, "./NumberOfDatasets", static_cast<int>(this->m_dataItemList.size()));

    unsigned num{ 0 };
    for (const auto & item : m_dataItemList)
    {
      std::ostringstream ss;
      ss << "./dataset-" << num;

      Group dsGroup = file.createGroup(ss.str().c_str());
      item->Save(dsGroup);

      num++;
    }
  }
  catch (const H5::Exception & e)
  {
    std::string msg =
      std::string("an exception occurred while writing data matrix to HDF5 file \n") + e.getCDetailMsg();
    STATISMO_LOG_ERROR(msg);
    throw StatisticalModelException(msg.c_str(), Status::IO_ERROR);
  }
}

template <typename T, typename Derived>
typename DataManagerBase<T, Derived>::DataItemListType
DataManagerBase<T, Derived>::GetData() const
{
  return m_dataItemList;
}

template <typename T, typename Derived>
typename DataManagerBase<T, Derived>::CrossValidationFoldListType
DataManagerBase<T, Derived>::GetCrossValidationFolds(unsigned nFolds, bool isRandomized) const
{
  STATISMO_LOG_INFO("Retrieving validation folds");
  STATISMO_LOG_INFO("Folds count: " + std::to_string(nFolds));
  STATISMO_LOG_INFO("Sample count: " + std::to_string(GetNumberOfSamples()));

  if (nFolds <= 1 || nFolds > GetNumberOfSamples())
  {
    throw StatisticalModelException("Invalid number of folds specified in GetCrossValidationFolds",
                                    Status::BAD_INPUT_ERROR);
  }

  auto nElemsPerFold = GetNumberOfSamples() / nFolds;

  // we create a vector with as many entries as datasets. Each entry contains the
  // fold the entry belongs to
  std::vector<std::size_t> batchAssignment(GetNumberOfSamples());

  for (std::size_t i = 0; i < GetNumberOfSamples(); i++)
  {
    batchAssignment[i] = std::min(i / nElemsPerFold, static_cast<std::size_t>(nFolds));
  }

  // randomly shuffle the vector
  if (isRandomized)
  {
    std::random_device rd;
    std::mt19937       g(rd());
    std::shuffle(std::begin(batchAssignment), std::end(batchAssignment), g);
  }

  // now we create the folds
  CrossValidationFoldListType foldList;
  for (unsigned currentFold = 0; currentFold < nFolds; currentFold++)
  {
    DataItemListType trainingData;
    DataItemListType testingData;

    unsigned sampleNum{ 0 };
    for (const auto & item : m_dataItemList)
    {
      if (batchAssignment[sampleNum] != currentFold)
      {
        trainingData.push_back(item);
      }
      else
      {
        testingData.push_back(item);
      }
      ++sampleNum;
    }
    foldList.emplace_back(trainingData, testingData);
  }
  return foldList;
}

template <typename T, typename Derived>
typename DataManagerBase<T, Derived>::CrossValidationFoldListType
DataManagerBase<T, Derived>::GetLeaveOneOutCrossValidationFolds() const
{
  STATISMO_LOG_INFO("Retrieving validation folds");
  CrossValidationFoldListType foldList;
  for (unsigned currentFold = 0; currentFold < GetNumberOfSamples(); currentFold++)
  {
    DataItemListType trainingData;
    DataItemListType testingData;

    unsigned sampleNum{ 0 };
    for (const auto & item : m_dataItemList)
    {
      if (sampleNum == currentFold)
      {
        testingData.push_back(item);
      }
      else
      {
        trainingData.push_back(item);
      }
      ++sampleNum;
    }
    CrossValidationFoldType fold(trainingData, testingData);
    foldList.push_back(fold);
  }
  return foldList;
}

template <typename T>
void
BasicDataManager<T>::AddDataset(DatasetConstPointerType dataset, const std::string & uri)
{
  STATISMO_LOG_INFO("Adding dataset with uri " + uri);

  auto sample = this->m_representer->CloneDataset(dataset);
  auto uw = MakeStackUnwinder([&]() { this->m_representer->DeleteDataset(sample); });

  this->m_dataItemList.push_back(MakeSharedPointer<DataItemType>(
    BasicDataItemType::Create(this->m_representer.get(), uri, this->m_representer->SampleToSampleVector(sample))));
}

} // Namespace statismo

#endif
