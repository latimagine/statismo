/*
 * This file is part of the statismo library.
 *
 * Author: Christoph Jud (christoph.jud@unibas.ch)
 *
 * Copyright (c) 2015 University of Basel
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
#include "statismo/core/RandUtils.h"
#include "statismo/core/PCAModelBuilder.h"
#include "statismo/VTK/vtkStandardMeshRepresenter.h"
#include "vtkTestHelper.h"

#include <Eigen/Geometry>

#include <vtkMath.h>
#include <vtkPoints.h>
#include <vtkPolyDataReader.h>
#include <vtkPolyDataWriter.h>
#include <vtkVersion.h>

#include <memory>
#include <string>
#include <vector>

using namespace statismo;
using namespace statismo::test;

namespace {
  using RepresenterType = vtkStandardMeshRepresenter              ;
  using DataManagerType = statismo::BasicDataManager<vtkPolyData> ;
  using PointType = vtkStandardMeshRepresenter::PointType   ;
  using DomainType = vtkStandardMeshRepresenter::DomainType  ;
  using DomainPointsListType = DomainType::DomainPointsListType        ;
  using StatisticalModelType = statismo::StatisticalModel<vtkPolyData> ;
  using PCAModelBuilderType = statismo::PCAModelBuilder<vtkPolyData> ;

  std::vector<std::string> _s_filenames;
  std::string _s_outdir;
  unsigned _s_pointCount;
  auto _s_rg = rand::RandGen(0);

/**
 * The test works as follows:
 *  - First a standard PCA model is built out of the sample hand-shapes of statismo
 *    It is ensured that the standard argument results in exactly the same model
 *    as if JacobiSVD is provided.
 *  - In the second test, a PCA model is built out of 5000 samples drawn from the
 *    previous built model. Here, the SelfAdjointEigenSolver is used, since there
 *    are more samples than variables.
 *
 * Arguments:
 *  - one can provide a number of points with which the samples should be subsampled (default is 100)
 *  - additionally, one can provide an output directory. If this is provided, the principal
 *    components of the model are sampled and stored in this directory. This is meant for visual inspection.
 */

int  TestBuildModel()
{
  auto reference = ReducePoints(LoadPolyData(_s_filenames[0]), _s_pointCount);
  auto representer = RepresenterType::SafeCreate(reference);
  auto dataManager = DataManagerType::SafeCreate(representer.get());

  for (const auto& f : _s_filenames) {
    dataManager->AddDataset(ReducePoints(LoadPolyData(f), _s_pointCount), "dataset");
  }

  const double                                         dataNoise = 0.0;

  // ----------------------------------------------------------
  // First compute PCA model using standard JacobiSVD
  // ----------------------------------------------------------
  auto                                           pcaModelBuilder = PCAModelBuilderType::SafeCreate();

  // perform with standard argument
  auto       jacobiModel = pcaModelBuilder->BuildNewModel(dataManager->GetData(), dataNoise, false);
  VectorType variance1 = jacobiModel->GetPCAVarianceVector();
  MatrixType pcbasis1 = jacobiModel->GetPCABasisMatrix();

  // perform with providing JacobiSVD
  jacobiModel =
    pcaModelBuilder->BuildNewModel(dataManager->GetData(), dataNoise, false, PCAModelBuilderType::JacobiSVD);

  VectorType variance2 = jacobiModel->GetPCAVarianceVector();
  MatrixType pcbasis2 = jacobiModel->GetPCABasisMatrix();

  STATISMO_ASSERT_DOUBLE_EQ(CompareVectors(variance1, variance2), 0.0f);
  STATISMO_ASSERT_DOUBLE_EQ(CompareMatrices(pcbasis1, pcbasis2), 0.0);

  // ----------------------------------------------------------
  // Generate a lot of samples out of the JacobiSVD model
  // ----------------------------------------------------------
  auto dataManager2 = DataManagerType::SafeCreate(representer.get());
  dataManager2->AddDataset(reference, "ref");
  for (unsigned i = 0; i < 5000; i++)
  {
    std::stringstream ss;
    ss << "sample" << i;
    dataManager2->AddDataset(jacobiModel->DrawSample(), ss.str().c_str());
  }

  // ----------------------------------------------------------
  // Compute PCA model using the SelfAdjointEigenSolver
  // ----------------------------------------------------------
  auto saesModel = pcaModelBuilder->BuildNewModel(
    dataManager2->GetData(), dataNoise, false, PCAModelBuilderType::SelfAdjointEigenSolver);

  // ----------------------------------------------------------
  // Comparing the models
  // - compare each principal component
  // ----------------------------------------------------------
  double error{0.0};
  for (unsigned i = 0;
       i < std::min(jacobiModel->GetNumberOfPrincipalComponents(), saesModel->GetNumberOfPrincipalComponents());
       i++)
  {
    VectorType coeff1 = VectorType::Zero(jacobiModel->GetNumberOfPrincipalComponents());
    coeff1[i] = 2;
    VectorType coeff2 = VectorType::Zero(saesModel->GetNumberOfPrincipalComponents());
    coeff2[i] = 2;

    // it might be, that the direction of the pc is in the opposite direction
    double equalDir =
      CompareVectors(jacobiModel->DrawSampleVector(coeff1, false), saesModel->DrawSampleVector(coeff2, false));
    coeff2[i] = -2;
    double oppositeDir =
      CompareVectors(jacobiModel->DrawSampleVector(coeff1, false), saesModel->DrawSampleVector(coeff2, false));

    error += std::min(equalDir, oppositeDir);

    if (_s_outdir.size() > 0)
    {
      std::stringstream ss1;
      ss1 << _s_outdir << "/jacobi-" << i << ".vtk";
      WritePolyData(jacobiModel->DrawSample(coeff1, false), ss1.str().c_str());

      std::stringstream ss2;
      ss2 << _s_outdir << "/saes-" << i << ".vtk";
      if (equalDir < oppositeDir)
        coeff2[i] = 2;
      WritePolyData(saesModel->DrawSample(coeff2, false), ss2.str().c_str());
    }
  }

  STATISMO_ASSERT_LT(error, 150.0);

  double varError{0.0};
  for (unsigned i = 0;
       i < std::min(jacobiModel->GetNumberOfPrincipalComponents(), saesModel->GetNumberOfPrincipalComponents());
       i++)
  {
    varError += std::sqrt(std::pow(jacobiModel->GetPCAVarianceVector()[i] - saesModel->GetPCAVarianceVector()[i], 2));
  }

  STATISMO_ASSERT_LT(varError, 250.0);

  return EXIT_SUCCESS;
}

} // namespace

int
PCAModelBuilderWithSelfAdjointEigenSolverTest(int argc, char ** argv)
{
  if (argc < 2)
  {
    std::cout << "Usage: " << argv[0] << " datadir "
              << "number_of_points(default=100) _s_outdir" << std::endl;
    exit(EXIT_FAILURE);
  }
  std::string datadir = argv[1];

  // number of points which are used for building the model
  // the number of points is reduced since the SelfAdjointEigenSolver operates
  // on the full covariance matrix.
  _s_pointCount = 100;
  if (argc == 3) {
    _s_pointCount = std::atoi(argv[2]);
  }

  if (argc == 4) {
    _s_outdir = argv[3];
  }

  for (unsigned i = 0; i <= 16; ++i) {
    _s_filenames.emplace_back(datadir + "/hand_polydata/hand-" + std::to_string(i) + ".vtk");
  }

  auto res = statismo::Translate([]() {
    return statismo::test::RunAllTests("PCAModelBuilderWithSelfAdjointEigenSolverTest", { { "TestBuildModel", TestBuildModel }});
  });

  return !CheckResultAndAssert(res, EXIT_SUCCESS);
}
