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
#include "statismo/core/GenericRepresenterValidator.h"
#include "statismo/ITK/itkStandardMeshRepresenter.h"

#include <itkMeshFileReader.h>

namespace {

constexpr unsigned                                          Dimensions = 3;
using MeshType = itk::Mesh<float, Dimensions>                    ;

std::string _s_dataDir;

MeshType::Pointer
_LoadMesh(const std::string & filename)
{
  auto reader = itk::MeshFileReader<MeshType>::New();
  reader->SetFileName(filename);
  reader->Update();
  MeshType::Pointer mesh = reader->GetOutput();
  mesh->DisconnectPipeline();
  return mesh;
}
}

int
TestRepresenterForMesh() {

  using RepresenterType =  itk::StandardMeshRepresenter<float, Dimensions> ;
using RepresenterValidatorType = GenericRepresenterValidator<RepresenterType> ;
  auto referenceFilename = _s_dataDir + "/hand_polydata/hand-0.vtk";
  auto testDatasetFilename = _s_dataDir + "/hand_polydata/hand-1.vtk";

  auto representer = RepresenterType::New();
  auto      reference = _LoadMesh(referenceFilename);
  representer->SetReference(reference);

  // choose a test dataset, a point and its associate pixel value

  auto testDataset = _LoadMesh(testDatasetFilename);
  unsigned            testPtId = 0;
  MeshType::PointType testPt;
  reference->GetPoint(testPtId, &testPt);
  MeshType::PointType testValue;
  testDataset->GetPoint(testPtId, &testValue);
  RepresenterValidatorType validator(representer, testDataset, std::make_pair(testPt, testValue));

  return (validator.RunAllTests() ? EXIT_SUCCESS : EXIT_FAILURE);
}

int
itkStandardMeshRepresenterTest(int argc, char ** argv)
{
  if (argc < 2)
  {
    std::cout << "Usage: " << argv[0] << " datadir" << std::endl;
    exit(EXIT_FAILURE);
  }
  _s_dataDir = argv[1];

  auto res = statismo::Translate([]() {
    return statismo::test::RunAllTests("itkStandardMeshRepresenterTest",
                                       { { "TestRepresenterForMesh", TestRepresenterForMesh }

                                       });
  });

  return !statismo::CheckResultAndAssert(res, EXIT_SUCCESS);
}
