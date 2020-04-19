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

#ifndef __STATIMO_CORE_MODEL_INFO_H_
#define __STATIMO_CORE_MODEL_INFO_H_

#include "statismo/core/CommonTypes.h"
#include "statismo/core/StatismoCoreExport.h"

#include <H5Cpp.h>

#include <list>
#include <vector>

namespace statismo
{

class BuilderInfo; // forward declaration

/**
 * \brief Store meta information about the model
 *
 * The ModelInfo object stores the scores and a list of BuilderInfo objects.
 * Each BuilderInfo contains the information (datasets, parameters) that were
 * used to build the model. If n model builders had been used in succession to create
 * a model, there will be a list of n builder objects.
 *
 * \ingroup Core
 */
class ModelInfo final
{
public:
  using BuilderInfoList = std::vector<BuilderInfo>;

  STATISMO_CORE_EXPORT
  ModelInfo() = default;

  /**
   * \brief Ctor
   * \param scores matrix holding the scores
   * \param builderInfos list of BuilderInfo objects
   */
  STATISMO_CORE_EXPORT
  ModelInfo(MatrixType scores, BuilderInfoList builderInfos);

  /**
   * \brief Ctor
   * \param scores matrix holding the scores
   */
  STATISMO_CORE_EXPORT explicit ModelInfo(MatrixType scores);

  /**
   * \brief Get all builder info
   */
  STATISMO_CORE_EXPORT BuilderInfoList
                       GetBuilderInfoList() const;

  /**
   * \brief Get the score matrix
   * \return a matrix where the i-th column corresponds to the
   * coefficients of the i-th dataset in the model
   */
  STATISMO_CORE_EXPORT const MatrixType &
                             GetScoresMatrix() const;

  /**
   * \brief Save model info in an hdf5 group
   */
  STATISMO_CORE_EXPORT void
  Save(const H5::H5Location & publicFg) const;

  /**
   * \brief Load model info from tan hdf5 group
   */
  STATISMO_CORE_EXPORT void
  Load(const H5::H5Location & publicFg);

private:
  BuilderInfo
  LoadDataInfoOldStatismoFormat(const H5::H5Location & publicModelGroup) const;

  MatrixType      m_scores;
  BuilderInfoList m_builderInfo;
};

/**
 * \brief Store meta information about the model
 *
 * Meta information consists of:
 *  - the name (uri) of the datasets used to build the models,
 *  - specific parameters of the builders
 *
 * \ingroup Core
 */
class BuilderInfo final
{
  friend class ModelInfo;

public:
  using KeyValuePair = std::pair<std::string, std::string>;
  using KeyValueList = std::list<KeyValuePair>;

  // Currently all the info entries are just simple list of string pairs.
  // We don't want to use maps, as this would sort the items according to the key.
  using DataInfoList = KeyValueList;
  using ParameterInfoList = KeyValueList;

  STATISMO_CORE_EXPORT
  BuilderInfo(std::string modelBuilderName, std::string buildTime, DataInfoList di, ParameterInfoList pi);

  STATISMO_CORE_EXPORT
  BuilderInfo(std::string modelBuilderName, DataInfoList di, ParameterInfoList pi);

  STATISMO_CORE_EXPORT
  BuilderInfo() = default;

  /**
   * \brief Save builder info to an hdf5 group
   */
  STATISMO_CORE_EXPORT void
  Save(const H5::H5Location & modelBuilderGroup) const;

  /**
   * \brief Load builder info from an hdf5 group
   */
  STATISMO_CORE_EXPORT void
  Load(const H5::H5Location & modelBuilderGroup);

  /**
   * \brief Get the data info
   */
  STATISMO_CORE_EXPORT const DataInfoList &
                             GetDataInfo() const;

  /**
   * \brief Get the parameter info
   */
  STATISMO_CORE_EXPORT const ParameterInfoList &
                             GetParameterInfo() const;

private:
  static void
  FillKeyValueListFromInfoGroup(const H5::H5Location & group, KeyValueList & keyValueList);

  std::string       m_modelBuilderName;
  std::string       m_buildtime;
  DataInfoList      m_dataInfo;
  ParameterInfoList m_parameterInfo;
};

} // namespace statismo

#endif
