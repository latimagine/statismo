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

#include "statismo/ITK/itkDataManager.h"
#include "statismo/ITK/itkPCAModelBuilder.h"
#include "statismo/ITK/itkStandardMeshRepresenter.h"
#include "statismo/ITK/itkIO.h"
#include "statismo/ITK/itkStatisticalModel.h"
#include "statismo/ITK/itkUtils.h"

#include <itkMesh.h>
#include <itkMeshFileWriter.h>
#include <itkMeshFileReader.h>


#include <iostream>

/*
 * This example shows the ITK Wrapping of statismo can be used to build a shape model.
 */

namespace
{
constexpr unsigned gk_dimensions = 3;
using MeshType = itk::Mesh<float, gk_dimensions>;
using RepresenterType = itk::StandardMeshRepresenter<float, gk_dimensions>;

void
DoRunExample(const char * referenceFilename, const char * dir, const char * modelname)
{
  using ModelBuilderType = itk::PCAModelBuilder<MeshType>;
  using DataManagerType = itk::DataManager<MeshType>;
  using MeshReaderType = itk::MeshFileReader<MeshType>;

  auto representer = RepresenterType::New();
  auto refReader = MeshReaderType::New();
  refReader->SetFileName(referenceFilename);
  refReader->Update();
  representer->SetReference(refReader->GetOutput());

  auto filenames = statismo::itk::GetDirFiles(dir, ".vtk");

  auto dataManager = DataManagerType::New();
  dataManager->SetRepresenter(representer);

  for (const auto & file : filenames)
  {
    auto fullpath = std::string(dir) + "/" + file;

    auto reader = MeshReaderType::New();
    reader->SetFileName(fullpath.c_str());
    reader->Update();

    MeshType::Pointer mesh = reader->GetOutput();
    dataManager->AddDataset(mesh, fullpath.c_str());
  }

  auto pcaModelBuilder = ModelBuilderType::New();
  auto model = pcaModelBuilder->BuildNewModel(dataManager->GetData(), 0);
  itk::StatismoIO<MeshType>::SaveStatisticalModel(model, modelname);
}
} // namespace

int
main(int argc, char * argv[])
{
  if (argc < 4)
  {
    std::cerr << "usage " << argv[0] << " referenceShape shapeDir modelname" << std::endl;
    return 1;
  }

  const char * reference = argv[1];
  const char * dir = argv[2];
  const char * modelname = argv[3];

  DoRunExample(reference, dir, modelname);

  std::cout << "Model building is completed successfully." << std::endl;
  return 0;
}
