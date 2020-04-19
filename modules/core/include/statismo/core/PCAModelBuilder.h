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

#ifndef __STATIMO_CORE_PCA_MODEL_BUILDER_H_
#define __STATIMO_CORE_PCA_MODEL_BUILDER_H_

#include "statismo/core/CommonTypes.h"
#include "statismo/core/Config.h"
#include "statismo/core/DataManager.h"
#include "statismo/core/ModelBuilder.h"
#include "statismo/core/ModelInfo.h"
#include "statismo/core/StatisticalModel.h"

#include <memory>
#include <vector>

namespace statismo
{
/**
 * \brief Create StatisticalModel using Principal Component Analysis.
 *
 * This class implements the classical PCA based approach to Statistical Models.
 *
 * \ingroup ModelBuilders
 * \ingroup Core
 */
template <typename T>
class PCAModelBuilder : public ModelBuilderBase<T, PCAModelBuilder<T>>
{

public:
  using Superclass = ModelBuilderBase<T, PCAModelBuilder<T>>;
  using DataManagerType = typename Superclass::DataManagerType;
  using StatisticalModelType = typename Superclass::StatisticalModelType;
  using DataItemListType = typename DataManagerType::DataItemListType;

  friend typename Superclass::ObjectFactoryType;

  /**
   * \brief Decomposition method resp. eigenvalue solver
   */
  enum class EigenValueMethod
  {
    JACOBI_SVD,               /**< most accurate but quite slow for larger systems */
    SELF_ADJOINT_EIGEN_SOLVER /**< more appropriate for large systems (especially, if there are more examples than
                                 variables) */
  };

  virtual ~PCAModelBuilder() = default; // NOLINT

  /**
   * \brief Build a new model from the training data provided in the dataManager
   * \param sampleDataList sample set holding the data
   * \param noiseVariance variance of N(0, noiseVariance) distributed noise on the points
   * If this parameter is set to 0, we have a standard PCA model. For values > 0 we have a PPCA model.
   * \param computeScores determines whether the scores (the pca coefficients of the examples) are computed and stored
   * as model info (computing the scores may take a long time for large models).
   * \param method Specifies the method which is used for the decomposition
   */
  UniquePtrType<StatisticalModelType>
  BuildNewModel(const DataItemListType & sampleDataList,
                double                   noiseVariance,
                bool                     computeScores = true,
                EigenValueMethod         method = EigenValueMethod::JACOBI_SVD) const;


private:
  PCAModelBuilder() = default;

  UniquePtrType<StatisticalModelType>
  BuildNewModelInternal(const Representer<T> * representer,
                        const MatrixType &     matX0,
                        const VectorType &     mu,
                        double                 noiseVariance,
                        EigenValueMethod       method = EigenValueMethod::JACOBI_SVD) const;
};

} // namespace statismo

#include "PCAModelBuilder.hxx"

#endif
