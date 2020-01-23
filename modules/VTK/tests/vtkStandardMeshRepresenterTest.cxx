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
#include "statismo/core/GenericRepresenterValidator.h"
#include "statismo/VTK/vtkStandardMeshRepresenter.h"

#include "vtkTestHelper.h"

#include <string>

using namespace statismo::test;

namespace {
std::string _s_dataDir;
}

int
TestRepresenterForMesh()
{
  using RepresenterType = statismo::vtkStandardMeshRepresenter;
  using RepresenterValidatorType = GenericRepresenterValidator<RepresenterType>;

  auto referenceFilename = _s_dataDir + "/hand_polydata/hand-0.vtk";
  auto testDatasetFilename = _s_dataDir + "/hand_polydata/hand-1.vtk";

  auto reference = LoadPolyData(referenceFilename);
  auto representer = RepresenterType::SafeCreate(reference);

  // choose a test dataset, a point (on the reference) and the associated point on the test example
  auto                 testDataset = LoadPolyData(testDatasetFilename);
  unsigned             testPtId{0};
  statismo::vtkPoint testPt(reference->GetPoints()->GetPoint(testPtId));
  statismo::vtkPoint testValue(testDataset->GetPoints()->GetPoint(testPtId));

  RepresenterValidatorType validator(representer.get(), testDataset, std::make_pair(testPt, testValue));

  return (validator.RunAllTests() ? EXIT_SUCCESS : EXIT_FAILURE);
}

int
vtkStandardMeshRepresenterTest(int argc, char ** argv)
{
  if (argc < 2)
  {
    std::cout << "Usage: " << argv[0] << " datadir" << std::endl;
    exit(EXIT_FAILURE);
  }
  _s_dataDir = argv[1];

  auto res = statismo::Translate([]() {
    return statismo::test::RunAllTests("vtkStandardImageRepresenterTest",
                                       {
                                         { "TestRepresenterForMesh", TestRepresenterForMesh }
                                       });
  });

  return !statismo::CheckResultAndAssert(res, EXIT_SUCCESS);
}
