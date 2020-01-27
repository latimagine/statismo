/*
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

#include "utils/statismoBuildModelUtils.h"

#include "statismo/ITK/itkUtils.h"
#include "statismo/ITK/itkDataManager.h"
#include "statismo/ITK/itkPCAModelBuilder.h"
#include "statismo/ITK/itkStandardMeshRepresenter.h"
#include "statismo/ITK/itkIO.h"
#include "statismo/ITK/itkStatisticalModel.h"

#include "lpo.h"

#include <itkImage.h>
#include <itkLandmarkBasedTransformInitializer.h>
#include <itkMesh.h>
#include <itkMeshFileReader.h>
#include <itkMeshFileWriter.h>
#include <itkRigid3DTransform.h>
#include <itkTransformMeshFilter.h>

namespace po = lpo;
using namespace std;

namespace
{

struct ProgramOptions
{
  string strDataListFile;
  string strProcrustesMode;
  string strProcrustesReferenceFile;
  string strOutputFileName;
  float  fNoiseVariance{ 0.0 };
};

bool
IsOptionsConflictPresent(ProgramOptions & opt)
{
  statismo::utils::ToLower(opt.strProcrustesMode);

  return (opt.strProcrustesMode != "reference" && opt.strProcrustesMode != "gpa") ||
         (opt.strProcrustesMode == "reference" && opt.strProcrustesReferenceFile.empty()) ||
         (opt.strProcrustesMode == "gpa" && !opt.strProcrustesReferenceFile.empty()) || opt.strDataListFile.empty() ||
         opt.strOutputFileName.empty() || opt.strDataListFile == opt.strOutputFileName ||
         (opt.strProcrustesMode == "reference" && (opt.strDataListFile == opt.strProcrustesReferenceFile ||
                                                   opt.strOutputFileName == opt.strProcrustesReferenceFile));
}

void
BuildAndSaveShapeModel(const ProgramOptions & opt)
{
  const unsigned kDimensions = 3;

  using RepresenterType = itk::StandardMeshRepresenter<float, kDimensions>;
  using MeshType = itk::Mesh<float, kDimensions>;
  using DataManagerType = itk::DataManager<MeshType>;
  using MeshReaderType = itk::MeshFileReader<MeshType>;
  using MeshReaderList = vector<MeshReaderType::Pointer>;
  using Rigid3DTransformType = itk::VersorRigid3DTransform<float>;
  using ImageType = itk::Image<float, kDimensions>;
  using LandmarkBasedTransformInitializerType =
    itk::LandmarkBasedTransformInitializer<Rigid3DTransformType, ImageType, ImageType>;
  using PCAModelBuilder = itk::PCAModelBuilder<MeshType>;
  ;
  using FilterType = itk::TransformMeshFilter<MeshType, MeshType, Rigid3DTransformType>;

  auto representer = RepresenterType::New();
  auto dataManager = DataManagerType::New();
  auto fileNames = statismo::cli::GetFileList(opt.strDataListFile);

  MeshReaderList meshReaders;
  meshReaders.reserve(fileNames.size());
  for (const auto & file : fileNames)
  {
    auto reader = MeshReaderType::New();
    reader->SetFileName(file);
    reader->Update();
    // itk::PCAModelBuilder is not a Filter in the ITK world, so the pipeline would not get executed if its main method
    // is called. So the pipeline before calling itk::PCAModelBuilder must be executed by the means of calls to Update()
    // (at least for last elements needed by itk::PCAModelBuilder).
    meshReaders.push_back(reader);
  }

  if (meshReaders.empty())
  {
    itkGenericExceptionMacro(<< "The specified data-list is empty.");
  }

  if (opt.strProcrustesMode == "reference")
  {
    auto refReader = MeshReaderType::New();
    refReader->SetFileName(opt.strProcrustesReferenceFile);
    refReader->Update();
    representer->SetReference(refReader->GetOutput());
  }
  else
  {
    vector<MeshType::Pointer> originalMeshes;
    for (const auto & reader : meshReaders)
    {
      originalMeshes.emplace_back(reader->GetOutput());
    }

    const unsigned kMaxGPAIterations = 20;
    const unsigned kNumberOfPoints = 100;
    const float    kBreakIfChangeBelow = 0.001f;

    auto referenceMesh = statismo::cli::
      CalculateProcrustesMeanMesh<MeshType, LandmarkBasedTransformInitializerType, Rigid3DTransformType, FilterType>(
        originalMeshes, kMaxGPAIterations, kNumberOfPoints, kBreakIfChangeBelow);
    representer->SetReference(referenceMesh);
  }

  dataManager->SetRepresenter(representer);
  for (const auto & reader : meshReaders)
  {
    dataManager->AddDataset(reader->GetOutput(), reader->GetFileName());
  }

  auto pcaModelBuilder = PCAModelBuilder::New();
  auto model = pcaModelBuilder->BuildNewModel(dataManager->GetData(), opt.fNoiseVariance);
  itk::StatismoIO<MeshType>::SaveStatisticalModel(model, opt.strOutputFileName);
}
} // namespace

int
main(int argc, char ** argv)
{
  ProgramOptions                          poParameters;
  po::program_options<std::string, float> parser{ argv[0], "Program help:" };

  parser
    .add_opt<std::string>({ "data-list",
                            "l",
                            "File containing a list of meshes to build shape model from",
                            &poParameters.strDataListFile,
                            "" },
                          true)
    .add_opt<std::string>({ "procrustes",
                            "p",
                            "Specify how the data is aligned: REFERENCE aligns all datasets rigidly to the reference "
                            "and GPA alignes all datasets to the population mean.",
                            &poParameters.strProcrustesMode,
                            "GPA" })
    .add_opt<std::string>(
      { "reference",
        "r",
        "Specify the reference used for model building. This is needed if --procrustes is REFERENCE",
        &poParameters.strProcrustesReferenceFile,
        "" })
    .add_opt<float>({ "noise", "n", "Noise variance of the PPCA model", &poParameters.fNoiseVariance, 0.0f, 0.0f })
    .add_pos_opt<std::string>({ "Name of the output file", &poParameters.strOutputFileName });

  if (!parser.parse(argc, argv))
  {
    return EXIT_FAILURE;
  }

  if (IsOptionsConflictPresent(poParameters))
  {
    cerr << "A conflict in the options exists or insufficient options were set." << endl;
    cout << parser << endl;
    return EXIT_FAILURE;
  }

  try
  {
    BuildAndSaveShapeModel(poParameters);
  }
  catch (const ifstream::failure & e)
  {
    cerr << "Could not read the data-list:" << endl;
    cerr << e.what() << endl;
    return EXIT_FAILURE;
  }
  catch (itk::ExceptionObject & e)
  {
    cerr << "Could not build the model:" << endl;
    cerr << e.what() << endl;
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
