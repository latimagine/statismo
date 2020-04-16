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
#ifndef __STATIMO_CORE_STATISTICAL_MODEL_H_
#define __STATIMO_CORE_STATISTICAL_MODEL_H_

#include "statismo/core/CommonTypes.h"
#include "statismo/core/Config.h"
#include "statismo/core/DataManager.h"
#include "statismo/core/ModelInfo.h"
#include "statismo/core/Representer.h"
#include "statismo/core/GenericFactory.h"
#include "statismo/core/NonCopyable.h"
#include "statismo/core/Logger.h"

#include <limits>
#include <vector>

namespace statismo
{

/**
 * \brief Representation of a linear statistical model (PCA Model).
 *
 * The statistical model class represents a statistical (PCA based) model.
 * The implementation is based on the Probabilistic PCA, which includes the Standard PCA as a special case.
 *
 * Mathematically, the statistical model is a generative model, where a sample is given as a linear combination
 * of a \f$n\f$ dimensional latent variable \f$ \alpha \f$ plus additive gaussian noise.
 * \f[ S = \mu + W \alpha + \epsilon. \f]
 * Here, \f$ \mu \in \mathbf{R}^p \f$ is the mean, \f$W \in \mathbf{R}^{p \times n}\f$ is a linear mapping,
 * \f$\alpha \in \mathbf{R}^n \f$ is a vector of latent variables (later also referred to as coefficients)
 * and \f$\epsilon \sim \mathcal{N}(0, \sigma^2)\f$ is a noise component.
 * The linear mapping \f$ W \f$ is referred to as the PCABasis. It is usually given as \f$W = U D \f$ where \f$U\f$
 * where U is an orthonormal matrix and \f$D\f$ is a scaling matrix, referred to as PCAvariance. Usually, \f$U \in
 * \mathbf{R}^{p \times n}\f$ is the matrix of eigenvectors of the data covariance matrix and \f$D\f$ the corresponding
 * eigenvalues.
 *
 * While all the matrices and vectors defined above could be obtained directly, the main goal of this class
 * is to abstract from these technicalities, by providing a high level interface to shape models.
 * In this high level view, the model represents a multivariate normal distribution over the types defined by the
 * representer (which are typically either, surface meshes, point clouds, deformation fields or images). This class
 * provides the method to sample from this probability distribution, and to compute the probability of given samples
 * directly.
 *
 * \ingroup Core
 */
template <typename T>
class StatisticalModel
  : public GenericFactory<StatisticalModel<T>>
  , NonCopyable
{
public:
  using RepresenterType = Representer<T>;
  using DatasetPointerType = typename RepresenterType::DatasetPointerType;
  using DatasetConstPointerType = typename RepresenterType::DatasetConstPointerType;
  using ValueType = typename RepresenterType::ValueType;
  using PointType = typename RepresenterType::PointType;
  using DomainType = Domain<PointType>;
  friend GenericFactory<StatisticalModel<T>>;

  using PointValuePairType = std::pair<PointType, ValueType>;
  using PointIdValuePairType = std::pair<unsigned, ValueType>;
  using PointValueListType = std::list<PointValuePairType>;
  using PointIdValueListType = std::list<PointIdValuePairType>;

  // Maybe at some point, we can statically define a 3x3 resp. 2x3 matrix type.
  using PointCovarianceMatrixType = MatrixType;
  using PointValueWithCovariancePairType = std::pair<PointValuePairType, PointCovarianceMatrixType>;
  using PointValueWithCovarianceListType = std::list<PointValueWithCovariancePairType>;

  virtual ~StatisticalModel(); // NOLINT

  virtual void
  SetLogger(Logger * logger)
  {
    m_logger = logger;
  }

  /**
   * \brief Destroy the object.
   */
  void
  Delete()
  {
    delete this;
  }


  /**
   * \name General Info
   */
  ///@{

  /**
   * \brief Get the number of PCA components in the model
   */
  unsigned int
  GetNumberOfPrincipalComponents() const;


  /**
   * \brief Get model info object
   * \sa ModelInfo
   */
  const ModelInfo &
  GetModelInfo() const;
  ///@}


  /**
   * \name Sample from the model
   *
   * \warning Note that these methods return a new Sample. If the representer used returns naked pointers (i.e. not
   * smart pointers), the sample needs to be deleted manually with DeleteDataset.
   */
  ///@{

  /**
   * \brief Get the value of the given sample at a given point
   * \param sample sample
   * \param ptId point id where to evaluate the sample
   */
  ValueType
  EvaluateSampleAtPoint(DatasetConstPointerType sample, unsigned ptId) const;


  /**
   * \brief Get the value of the given sample corresponding to the given domain point
   * \param sample sample
   * \param point the (domain) point on which the sample should be evaluated
   */
  ValueType
  EvaluateSampleAtPoint(DatasetConstPointerType sample, const PointType & point) const;


  /**
   * \brief Get a new sample representing the mean of the model
   */
  DatasetPointerType
  DrawMean() const;


  /**
   * \brief Draw the sample with the given coefficients
   * \param coefficients coefficient vector. The size of the coefficient vector should be equal
   * to the number of factors in the model. Otherwise an exception is thrown.
   * \param addNoise if true, the Gaussian noise assumed in the model is added to the sample
   */
  DatasetPointerType
  DrawSample(const VectorType & coefficients, bool addNoise = false) const;


  /**
   * \brief Draw a sample with random coefficients
   * \see DrawSample
   */
  DatasetPointerType
  DrawSample(bool addNoise = false) const;


  /**
   * \brief Draw the sample corresponding to the ith pca matrix
   * \param pcaComponentIndex number of the PCA Basis to be retrieved
   */
  DatasetPointerType
  DrawPCABasisSample(unsigned pcaComponentIndex) const;


  /**
   * \name Point sampling and point information
   */
  ///@{

  /**
   * \brief Get the mean of the model evaluated at the given point
   * \param point point on the domain the model is defined
   */
  ValueType
  DrawMeanAtPoint(const PointType & point) const;


  /**
   * \brief Get the mean of the model evaluated at the given point index
   * \param pointId pointId of the point where it should be evaluated (as defined by the representer)
   */
  ValueType
  DrawMeanAtPoint(unsigned pointId) const;


  /**
   * \brief Get the value of the sample defined by coefficients at the specified point.
   * \param coefficients coefficients of the sample
   * \param point point of the sample where it is evaluated
   * \param addNoise if true, the Gaussian noise assumed in the model is added to the sample
   *
   * This method computes the value of the sample only for the given point, and is thus much more
   * efficient that calling DrawSample, if only a few points are of interest.
   */
  ValueType
  DrawSampleAtPoint(const VectorType & coefficients, const PointType & point, bool addNoise = false) const;


  /**
   * \brief Get the value of the sample defined by coefficients at the specified point index
   * \param coefficients coefficients of the sample
   * \param ptId point of the sample where it is evaluated
   * \param addNoise if true, the Gaussian noise assumed in the model is added to the sample
   *
   * This method computes the value of the sample only for the given point, and is thus much more
   * efficient that calling DrawSample, if only a few points are of interest.
   */
  ValueType
  DrawSampleAtPoint(const VectorType & coefficients, unsigned ptId, bool addNoise = false) const;


  /**
   * \brief Compute the jacobian of the Statistical model at a given point
   * \param pt point where the Jacobian is computed
   * \return Jacobian matrix
   */
  MatrixType
  GetJacobian(const PointType & pt) const;


  /**
   * \brief Compute the jacobian of the Statistical model at a specified point index
   * \param ptId pointID where the Jacobian is computed
   * \return Jacobian matrix
   */
  MatrixType
  GetJacobian(unsigned ptId) const;


  /**
   * \brief Get the variance in the model for point pt
   * \param pt1 point 1
   * \param pt2 point 2
   * \return a d x d covariance matrix
   */
  MatrixType
  GetCovarianceAtPoint(const PointType & pt1, const PointType & pt2) const;


  /**
   * \brief Get the variance in the model for point pt
   * \param ptId1 point 1
   * \param ptId2 point 2
   * \return a d x d covariance matrix
   */
  MatrixType
  GetCovarianceAtPoint(unsigned ptId1, unsigned ptId2) const;
  ///@}


  /**
   * \name Statistical Information from Dataset
   */
  ///@{

  /**
   * \brief Get the covariance matrix for the model
   *
   * If the model is defined on n points, in d dimensions, then this is a \f$nd \times nd\f$ matrix of
   * n \f$d \times d \f$ block matrices corresponding to the covariance at each point.
   *
   * \warning This method is only useful when $n$ is small, since otherwise the matrix
   * becomes huge.
   */
  MatrixType
  GetCovarianceMatrix() const;


  /**
   * \brief Get the probability of observing the given dataset under this model
   *
   * If the coefficients \f$\alpha \in \mathbf{R}^n\f$ define the dataset, the probability is
   * \f$
   * (2 \pi)^{- \frac{n}{2}} \exp(||\alpha||)
   * \f$
   */
  double
  ComputeProbability(DatasetConstPointerType dataset) const;


  /**
   * \brief Get the log probability of observing a given dataset.
   */
  double
  ComputeLogProbability(DatasetConstPointerType dataset) const;


  /**
   * \brief Get the probability of observing the given coefficients under this model.
   * \param coefficients coefficients \f$\alpha \in \mathbf{R}^n\f$
   *
   * If the coefficients \f$\alpha \in \mathbf{R}^n\f$ define the dataset, the probability is
   * \f$
   * (2 \pi)^{- \frac{n}{2}} \exp(||\alpha||)
   * \f$
   */
  double
  ComputeProbabilityOfCoefficients(const VectorType & coefficients) const;


  /**
   * \brief Get the log probability of observing given coefficients.
   * \param coefficients coefficients \f$\alpha \in \mathbf{R}^n\f$
   */
  double
  ComputeLogProbabilityOfCoefficients(const VectorType & coefficients) const;


  /**
   * \brief Get the mahalonoibs distance for the given dataset.
   */
  double
  ComputeMahalanobisDistance(DatasetConstPointerType dataset) const;


  /**
   * \brief Get the coefficients of the latent variables for the given dataset, i.e.
   * the vectors of numbers \f$\alpha \f$, such that for the dataset \f$S\f$ it holds that
   * \f$ S = \mu + U \alpha\f$
   * \return The coefficient vector \f$\alpha\f$
   */
  VectorType
  ComputeCoefficients(DatasetConstPointerType dataset) const;


  /**
   * \brief Get the coefficients of the latent variables for the given values provided in the PointValueList.
   * \param pointValueList A list with PointValuePairs .
   * \param pointValueNoiseVariance The variance of estimated (gaussian) noise at the known points
   *
   * This is useful, when only a part of the dataset is given.
   * The method is described in the paper \cite 5 .
   */
  VectorType
  ComputeCoefficientsForPointValues(const PointValueListType & pointValueList,
                                    double                     pointValueNoiseVariance = 0.0) const;


  /**
   * \brief Similar to ComputeCoefficientsForPointValues, only here there is no global pointValueNoiseVariance
   * \param pointValuesWithCovariance A list with PointValuePairs and PointCovarianceMatrices
   *
   * Instead, a covariance matrix with noise values is specified for each point.
   * The returned coefficients are the mean of the posterior model described in \cite 4 .
   *
   * To get the full posterior model, use the PosteriorModelBuilder
   */
  VectorType
  ComputeCoefficientsForPointValuesWithCovariance(
    const PointValueWithCovarianceListType & pointValuesWithCovariance) const;


  /**
   * \brief Version with point indices
   * \param pointIdValueList list with (Point,Value) pairs, a list of (PointId, Value) is provided
   * \param pointValueNoiseVariance variance of estimated (gaussian) noise at the known points
   */
  VectorType
  ComputeCoefficientsForPointIDValues(const PointIdValueListType & pointIdValueList,
                                      double                       pointValueNoiseVariance = 0.0) const;


  /**
   * \name Low level access
   * These methods provide a low level interface to the model content. They are of only limited use for
   * an application. Prefer whenever possible the high level functions.
   */
  ///@{

  /**
   * \brief Get the variance of the noise of the error term, that was set when the model was built.
   */
  float
  GetNoiseVariance() const;

  /**
   * \brief Get a vector where each element holds the variance of the corresponding principal component in data space
   */
  const VectorType &
  GetPCAVarianceVector() const;

  /**
   * \brief Get a vector holding the mean
   *
   * Assume the mean \f$\mu \subset \mathbf{R}^d\f$ is defined on
   * \f$p\f$ points, the returned mean vector \f$m\f$ has dimensionality \f$m \in \mathbf{R}^{dp} \f$, i.e.
   * the \f$d\f$ components are stacked into the vector. The order of the components in the vector is
   * undefined and depends on the representer.
   * */
  const VectorType &
  GetMeanVector() const;


  /**
   * \brief Get a matrix with the PCA Basis as its columns
   *
   * Assume the shapes \f$s \subset \mathbf{R}^d\f$ are defined on
   * \f$n\f$ points, the returned matrix \f$W\f$ has dimensionality \f$W \in \mathbf{R}^{dp \times n} \f$, i.e.
   * the \f$d\f$ components are stacked into the matrix. The order of the components in the matrix is
   * undefined and depends on the representer.
   */
  const MatrixType &
  GetPCABasisMatrix() const;


  /**
   * \brief Get the PCA Matrix, but with its principal axis normalized to unit length
   * \warning This is more expensive than GetPCABasisMatrix as the matrix has to be computed
   * and a copy is returned
   */
  MatrixType
  GetOrthonormalPCABasisMatrix() const;


  /**
   * \brief Get an instance for the given coefficients as a vector
   * \param coefficients coefficients of the sample
   * \param addNoise if true, the Gaussian noise assumed in the model is added to the sample
   */
  VectorType
  DrawSampleVector(const VectorType & coefficients, bool addNoise = false) const;
  ///@}


  ///@{
  /**
   * \brief Set the model information
   * \warning This is for library internal use only
   */
  void
  SetModelInfo(const ModelInfo & modelInfo);


  /**
   * \brief Compute the coefficients for the given sample vector
   * \warning This is for library internal use only
   */
  VectorType
  ComputeCoefficientsForSampleVector(const VectorType & sample) const;


  /**
   * \brief Get an instance of the representer
   */
  const RepresenterType *
  GetRepresenter() const
  {
    return m_representer;
  }


  /**
   * \brief Get the domain of the statistical model
   */
  const DomainType &
  GetDomain() const
  {
    return m_representer->GetDomain();
  }

  ///@}

protected:
  Logger *
  GetLogger() const
  {
    return m_logger;
  }

private:
  // computes the M Matrix for the PPCA Method (see Bishop, PRML, Chapter 12)
  void
  CheckAndUpdateCachedParameters() const;

  /**
   * \brief Create an instance of the StatisticalModel
   * \param representer an instance of the representer, used to convert the samples to dataset of the represented type.
   */
  StatisticalModel(const RepresenterType * representer,
                   VectorType              m,
                   const MatrixType &      orthonormalPCABasis,
                   VectorType              pcaVariance,
                   double                  noiseVariance);


  const RepresenterType * m_representer;
  VectorType              m_mean;
  MatrixType              m_pcaBasisMatrix;
  VectorType              m_pcaVariance;
  float                   m_noiseVariance;
  // caching
  mutable bool m_cachedValuesValid;
  // the matrix M^{-1} in Bishops PRML book. This is roughly the Latent Covariance matrix (but not exactly)
  mutable MatrixType m_matMInverse;
  ModelInfo          m_modelInfo;
  Logger *           m_logger{ nullptr };
};

} // namespace statismo

#include "StatisticalModel.hxx"

#endif
