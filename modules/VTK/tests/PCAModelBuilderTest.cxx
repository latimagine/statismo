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
#include "statismo/core/GenericRepresenterValidator.h"
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

namespace
{

using RepresenterType = vtkStandardMeshRepresenter;
using DataManagerType = statismo::BasicDataManager<vtkPolyData>;
using PointType = vtkStandardMeshRepresenter::PointType;
using DomainType = vtkStandardMeshRepresenter::DomainType;
using DomainPointsListType = DomainType::DomainPointsListType;
using StatisticalModelType = statismo::StatisticalModel<vtkPolyData>;
using PCAModelBuilderType = statismo::PCAModelBuilder<vtkPolyData>;

std::vector<std::string> _s_filenames;

bool
_TestBuildModel(unsigned pointsCount, const VectorType & baselineVariance)
{
  auto reference = ReducePoints(LoadPolyData(_s_filenames[0]), pointsCount);
  auto representer = RepresenterType::SafeCreate(reference);
  auto dataManager = DataManagerType::SafeCreate(representer.get());

  for (const auto & f : _s_filenames)
  {
    dataManager->AddDataset(ReducePoints(LoadPolyData(f), pointsCount), "dataset");
  }

  auto pcaModelBuilder = PCAModelBuilderType::SafeCreate();

  // perform with standard argument
  auto       PCAModel = pcaModelBuilder->BuildNewModel(dataManager->GetData(), 0, false);
  VectorType variance = PCAModel->GetPCAVarianceVector();

  // max. acceptable difference between expected and calculated values in percent
  VectorType::Scalar maxPermittedDifference = 0.1; // 1%

  STATISMO_ASSERT_LT(CompareVectors(variance, baselineVariance), maxPermittedDifference);

  return EXIT_SUCCESS;
}

int
TestBuildWithPGreaterThanN()
{
  VectorType baselineVariance(10);
  baselineVariance << 460.601104736328125, 211.22674560546875, 107.32666015625, 71.84774017333984375,
    36.4659576416015625, 22.3681926727294921875, 11.6593990325927734375, 4.789171695709228515625,
    1.28080332279205322265625, 0.77941668033599853515625;

  return _TestBuildModel(5, baselineVariance);
}

int
TestBuildWithNGreaterThanP()
{
  VectorType baselineVariance(16);
  baselineVariance << 5175.92236328125, 3022.61181640625, 1786.9608154296875, 1131.9517822265625, 727.96649169921875,
    480.115386962890625, 365.292266845703125, 233.9134063720703125, 173.226318359375, 164.652557373046875,
    128.6950531005859375, 91.76165008544921875, 80.23679351806640625, 69.49117279052734375, 50.3206024169921875,
    42.5595245361328125;

  return _TestBuildModel(100, baselineVariance);
}
} // namespace

int
PCAModelBuilderTest(int argc, char ** argv)
{
  if (argc < 2)
  {
    std::cout << "Usage: " << argv[0] << " datadir " << std::endl;
    return EXIT_FAILURE;
  }

  std::string datadir = argv[1];

  for (unsigned i = 0; i <= 16; ++i)
  {
    _s_filenames.emplace_back(datadir + "/hand_polydata/hand-" + std::to_string(i) + ".vtk");
  }

  auto res = statismo::Translate([]() {
    return statismo::test::RunAllTests("PCAModelBuilderTest",
                                       { { "TestBuildWithPGreaterThanN", TestBuildWithPGreaterThanN },
                                         { "TestBuildWithNGreaterThanP", TestBuildWithNGreaterThanP } });
  });

  return !CheckResultAndAssert(res, EXIT_SUCCESS);
}
