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

#include "StatismoUnitTest.h"
#include "statismo/core/Exceptions.h"

#include "statismo/core/CommonTypes.h"
#include "statismo/core/DataManager.h"
#include "statismo/core/Domain.h"
#include "statismo/core/PosteriorModelBuilder.h"
#include "statismo/core/PCAModelBuilder.h"
#include "statismo/VTK/vtkStandardMeshRepresenter.h"
#include "vtkTestHelper.h"

#include <Eigen/Geometry>

#include <vtkMath.h>
#include <vtkVersion.h>

#include <memory>
#include <string>

using namespace statismo;
using namespace statismo::test;

namespace {

    using RepresenterType =  vtkStandardMeshRepresenter              ;
  using DataManagerType =  statismo::BasicDataManager<vtkPolyData> ;
  using PointType =  vtkStandardMeshRepresenter::PointType   ;
  using DomainType =  vtkStandardMeshRepresenter::DomainType  ;
  using DomainPointsListType =  DomainType::DomainPointsListType        ;
  using StatisticalModelType =  statismo::StatisticalModel<vtkPolyData> ;

  using PosteriorModelBuilderType =  statismo::PosteriorModelBuilder<vtkPolyData>                ;
  using PointValuePairType =  PosteriorModelBuilderType::PointValuePairType               ;
  using PointValueWithCovariancePairType =  PosteriorModelBuilderType::PointValueWithCovariancePairType ;
  using PointValueWithCovarianceListType =  PosteriorModelBuilderType::PointValueWithCovarianceListType ;
  using MatrixType =  statismo::MatrixType;
    using PCAModelBuilderType =  statismo::PCAModelBuilder<vtkPolyData> ;
std::vector<std::string> _s_filenames;
}

int TestPosteriorMain() {
                                
  const unsigned nPointsFixed = 100;
  const unsigned nPointsTest = 1000;
  const double   tolerance = 0.01;
  const double                           pointValueNoiseVariance = 0.1;

  auto reference = LoadPolyData(_s_filenames[0]);
  auto representer = RepresenterType::SafeCreate(reference);
  auto dataManager = DataManagerType::SafeCreate(representer.get());

  //dataManager->AddDataset(reference, "ref");
  for (const auto& f : _s_filenames) {
    dataManager->AddDataset(LoadPolyData(f), "dataset");
  }

  const MatrixType                 pointCovarianceMatrix = pointValueNoiseVariance * MatrixType::Identity(3, 3);
  PointValueWithCovarianceListType pvcList;

  auto testSample = dataManager->GetData().back()->GetSample();

  auto domaintPointsList = representer->GetDomain().GetDomainPoints();
  unsigned             nDomainPoints = representer->GetDomain().GetNumberOfPoints();

  for (unsigned ptId  = 0; ptId  < nDomainPoints; ptId  = ptId  + nDomainPoints / nPointsFixed)
  {
    pvcList.emplace_back(PointValuePairType(domaintPointsList[ptId ], testSample->GetPoint(ptId )), pointCovarianceMatrix);
  }

  auto pModelBuilder = PosteriorModelBuilderType::SafeCreate();
  auto posteriorModel = pModelBuilder->BuildNewModel(dataManager->GetData(), pvcList, 0.1);

  auto posteriorMean  = posteriorModel->DrawMean();

  for (unsigned ptId  = 0; ptId  < nDomainPoints; ptId  = ptId  + nDomainPoints / nPointsTest)
  {
    STATISMO_ASSERT_LTE(vtkMath::Distance2BetweenPoints(posteriorMean->GetPoint(ptId ), testSample->GetPoint(ptId )), tolerance * tolerance);
  }

  auto                                           pcaModelBuilder = PCAModelBuilderType::SafeCreate();
  auto fullModel = pcaModelBuilder->BuildNewModel(dataManager->GetData(), 0.1, false);

  PointType middleFiducial(244, 290, 0);
  PointType funkyTargetPoint(270, 300, 0);

  statismo::VectorType anisotropicNoiseVariances(3);
  anisotropicNoiseVariances << .01, 100, .01; // 100, 100;

  // This assumes that the normal vectors is (1,0,0) and the tangential (0,1,0) and (0,0,1)
  MatrixType fiducialCovMatrix = anisotropicNoiseVariances.asDiagonal();

  PointValueWithCovarianceListType pointValueWithCovarianceList;
  pointValueWithCovarianceList.emplace_back(PointValuePairType{middleFiducial, funkyTargetPoint}, fiducialCovMatrix);

  auto anisotropicPosteriorModelBuilder = PosteriorModelBuilderType::SafeCreate();
  auto anisotropicPosteriorModel =
    anisotropicPosteriorModelBuilder->BuildNewModelFromModel(fullModel.get(), pointValueWithCovarianceList, false);

  posteriorMean = anisotropicPosteriorModel->DrawMean();

  STATISMO_ASSERT_GT(fullModel->ComputeLogProbability(posteriorMean), -10.0);

  VectorType coeffs = fullModel->ComputeCoefficientsForPointValuesWithCovariance(pointValueWithCovarianceList);
  VectorType coeffsFromPosteriorMean = fullModel->ComputeCoefficients(posteriorMean);

  STATISMO_ASSERT_LTE((coeffs - coeffsFromPosteriorMean).norm(), tolerance);

  return EXIT_SUCCESS;
}

int TestPosteriorOnePoint() {

  auto onePointPD = vtkSmartPointer<vtkPolyData>::New();
  auto onePoint = vtkSmartPointer<vtkPoints>::New();
  onePoint->Allocate(1);
  onePoint->InsertNextPoint(0, 0, 0);
  onePointPD->SetPoints(onePoint);
  auto       onePointRepresenter = RepresenterType::SafeCreate(onePointPD);
  VectorType onePointMean(3);
  onePointMean << 0, 0, 0;
  VectorType onePointVar(3);
  onePointVar << 1, 1, 1;
  MatrixType onePointPCABasis = MatrixType::Identity(3, 3);

  auto onePointModel =
    StatisticalModelType::SafeCreate(onePointRepresenter.get(), onePointMean, onePointPCABasis, onePointVar, 0.0);

  Eigen::Matrix3f rotMatrix;
  rotMatrix = Eigen::AngleAxisf(40, Eigen::Vector3f(0, 0, 1));

  VectorType covTestVector(3);
  covTestVector << 1, 0, 0;
  VectorType covTestVector1(3);
  covTestVector1 << 0, 2, 0;
  VectorType covTestVector2(3);
  covTestVector2 << 0, 0, 3;

  MatrixType testMatrix(3, 3);
  testMatrix.col(0) = covTestVector;
  testMatrix.col(1) = covTestVector1;
  testMatrix.col(2) = covTestVector2;

  testMatrix = rotMatrix * testMatrix;

  VectorType testValuePoint(3);
  testValuePoint << 1, 1, 1;
  testValuePoint = rotMatrix * testValuePoint;
  vtkPoint tvPoint(testValuePoint(0), testValuePoint(1), testValuePoint(2));

  MatrixType                       testCovMatrix = testMatrix * testMatrix.transpose();
  PointValuePairType               pvPair(vtkPoint(0, 0, 0), tvPoint);
  PointValueWithCovariancePairType pvcPair(pvPair, testCovMatrix);
  PointValueWithCovarianceListType onePointPvcList;
  onePointPvcList.push_back(pvcPair);

  auto onePointPosteriorModelBuilder = PosteriorModelBuilderType::SafeCreate();
  auto onePointPosteriorModel =
    onePointPosteriorModelBuilder->BuildNewModelFromModel(onePointModel.get(), onePointPvcList);

  VectorType posteriorModelMean = rotMatrix.inverse() * onePointPosteriorModel->GetMeanVector();
  VectorType knownMean(3);
  knownMean << 0.5, 0.2, 0.1;

  STATISMO_ASSERT_LTE((posteriorModelMean - knownMean).norm(), 0.00001);

  VectorType posteriorModelVariance = onePointPosteriorModel->GetPCAVarianceVector();
  VectorType knownVariance(3);
  knownVariance << 0.9, 0.8, 0.5;

  STATISMO_ASSERT_LTE((posteriorModelVariance - knownVariance).norm(), 0.00001);

  MatrixType rotatedOrthoPCAMatrix = rotMatrix.inverse() * onePointPosteriorModel->GetOrthonormalPCABasisMatrix();
  
  STATISMO_ASSERT_LTE((rotatedOrthoPCAMatrix * rotatedOrthoPCAMatrix.transpose() - MatrixType::Identity(3, 3)).norm(), 0.00001);

  return EXIT_SUCCESS;
}



int
PosteriorModelBuilderTest(int argc, char ** argv)
{
  if (argc < 2)
  {
    std::cout << "Usage: " << argv[0] << " datadir" << std::endl;
    exit(EXIT_FAILURE);
  }
  std::string datadir = std::string(argv[1]);

  for (unsigned i = 0; i <= 4; ++i) {
    _s_filenames.emplace_back(datadir + "/hand_polydata/hand-" + std::to_string(i) + ".vtk");
  }

  auto res = statismo::Translate([]() {
    return statismo::test::RunAllTests("PosteriorModelBuilderTest", { { "TestPosteriorMain", TestPosteriorMain },
    { "TestPosteriorOnePoint", TestPosteriorOnePoint } });
  });

  return !CheckResultAndAssert(res, EXIT_SUCCESS);

}
