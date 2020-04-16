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

#include "utils/statismoLoggingUtils.h"
#include "utils/itkPenalizingMeanSquaresPointSetToImageMetric.h"
#include "utils/statismoBuildPosteriorModelUtils.h"
#include "utils/statismoFittingUtils.h"

#include "statismo/ITK/itkReducedVarianceModelBuilder.h"
#include "statismo/ITK/itkStandardMeshRepresenter.h"
#include "statismo/ITK/itkIO.h"
#include "statismo/ITK/itkStatisticalModel.h"
#include "statismo/ITK/itkStatisticalShapeModelTransform.h"

#include "lpo.h"

#include <itkBoundingBox.h>
#include <itkCompositeTransform.h>
#include <itkSignedMaurerDistanceMapImageFilter.h>
#include <itkLBFGSOptimizer.h>
#include <itkLinearInterpolateImageFunction.h>
#include <itkMesh.h>
#include <itkPointSetToImageFilter.h>
#include <itkPointSetToImageRegistrationMethod.h>
#include <itkPointsLocator.h>
#include <itkVersorRigid3DTransform.h>

namespace po = lpo;
using namespace std;

namespace
{

constexpr unsigned gk_dimensions = 3;
using DataType = itk::Mesh<float, gk_dimensions>;
using DistanceImageType = itk::Image<float, gk_dimensions>;
using StatisticalModelType = itk::StatisticalModel<DataType>;

struct ProgramOptions
{
  bool bPrintFittingInformation{ false };

  string strInputModelFileName;
  string strInputTargetMeshFileName;

  string strOutputFittedMeshFileName;
  string strOutputProjectedMeshFileName;

  double   dRegularizationWeight{ 0.0 };
  unsigned uNumberOfIterations{ 0 };
  string   strInputFixedLandmarksFileName;
  string   strInputMovingLandmarksFileName;
  double   dLandmarksVariance{ 0.0 };

  bool        bIsQuiet{ false };
  std::string strLogFile{ "" };
};

bool
IsOptionsConflictPresent(const ProgramOptions & opt)
{
  return ((!opt.strInputFixedLandmarksFileName.empty()) ^ (!opt.strInputMovingLandmarksFileName.empty())) ||
         (opt.strOutputFittedMeshFileName.empty() && opt.strOutputProjectedMeshFileName.empty()) ||
         opt.strInputModelFileName.empty() || opt.strInputTargetMeshFileName.empty();
}

DistanceImageType::Pointer
ComputeDistanceImageForMesh(const DataType::Pointer & mesh, unsigned distImageResolution = 256)
{
  // Compute a bounding box around the reference shape
  using BoundingBoxType = itk::BoundingBox<int, gk_dimensions, float, DataType::PointsContainer>;
  using BinaryImageType = itk::Image<unsigned char, gk_dimensions>;
  using PointSetType = itk::PointSet<float, gk_dimensions>;
  using PointsToImageFilterType = itk::PointSetToImageFilter<PointSetType, BinaryImageType>;
  using DistanceFilterType = itk::SignedMaurerDistanceMapImageFilter<BinaryImageType, DistanceImageType>;

  auto boundingBox = BoundingBoxType::New();
  boundingBox->SetPoints(mesh->GetPoints());
  boundingBox->ComputeBoundingBox();

  // Compute a binary image from the point set, which is as large as the bounding box plus a margin.
  auto pointsToImageFilter = PointsToImageFilterType::New();
  pointsToImageFilter->SetInput(mesh);
  BinaryImageType::SpacingType spacing;
  BinaryImageType::SpacingType margin;
  BinaryImageType::PointType   origin = boundingBox->GetMinimum();
  BinaryImageType::SpacingType diff = boundingBox->GetMaximum() - boundingBox->GetMinimum();
  BinaryImageType::SizeType    size;

  for (unsigned i = 0; i < gk_dimensions; ++i)
  {
    margin[i] = diff[i] * 0.1; // 10 % margin on each side
    origin[i] -= margin[i];
    size[i] = distImageResolution + margin[i];
    spacing[i] = (diff[i] + 2.0 * margin[i]) / distImageResolution;
  }

  pointsToImageFilter->SetSpacing(spacing);
  pointsToImageFilter->SetOrigin(origin);
  pointsToImageFilter->SetSize(size);
  pointsToImageFilter->Update();

  // compute a distance map to the points in the pointset
  BinaryImageType::Pointer binaryImage = pointsToImageFilter->GetOutput();

  auto distanceFilter = DistanceFilterType::New();
  distanceFilter->SetInput(binaryImage);
  distanceFilter->Update();

  DistanceImageType::Pointer distImage = distanceFilter->GetOutput();
  return distImage;
}

void
SaveMesh(const DataType::Pointer & data, const string & strFileName)
{
  using DataWriterType = itk::MeshFileWriter<DataType>;
  auto writer = DataWriterType::New();
  writer->SetFileName(strFileName);
  writer->SetInput(data);
  writer->Update();
}

template <class PointsLocatorType>
DataType::Pointer
ProjectOnTargetMesh(const DataType::Pointer & mesh, const DataType::Pointer & targetMesh)
{
  auto ptLocator = PointsLocatorType::New();
  ptLocator->SetPoints(targetMesh->GetPoints());
  ptLocator->Initialize();

  auto projectedMesh = statismo::itk::CloneMesh<DataType>(mesh);
  for (DataType::PointsContainer::Iterator pointIter = projectedMesh->GetPoints()->Begin();
       pointIter != projectedMesh->GetPoints()->End();
       ++pointIter)
  {
    unsigned uClosestPointId = ptLocator->FindClosestPoint(pointIter->Value());
    pointIter->Value() = targetMesh->GetPoint(uClosestPointId);
  }
  return projectedMesh;
}

void
FitMesh(const ProgramOptions & opt, statismo::cli::ConsoleOutputSilencer * pCOSilencer, statismo::Logger * logger)
{
  using MeshReaderType = itk::MeshFileReader<DataType>;
  using RepresenterType = itk::StandardMeshRepresenter<float, gk_dimensions>;
  using StatisticalModelTransformType = itk::StatisticalShapeModelTransform<DataType, double, gk_dimensions>;
  using TransformType = itk::Transform<double, gk_dimensions, gk_dimensions>;
  using PointsLocatorType = itk::PointsLocator<DataType::PointsContainer>;
  using OptimizerType = itk::LBFGSOptimizer;
  using InterpolatorType = itk::LinearInterpolateImageFunction<DistanceImageType, double>;
  using PointSetType = itk::PointSet<float, gk_dimensions>;
  using MetricType = itk::PenalizingMeanSquaresPointSetToImageMetric<PointSetType, DistanceImageType>;
  using RegistrationFilterType = itk::PointSetToImageRegistrationMethod<PointSetType, DistanceImageType>;
  using TransformMeshFilterType = itk::TransformMeshFilter<DataType, DataType, TransformType>;

  auto meshReader = MeshReaderType::New();
  meshReader->SetFileName(opt.strInputTargetMeshFileName.c_str());
  meshReader->Update();

  DataType::Pointer targetMesh = meshReader->GetOutput();

  auto representer = RepresenterType::New();
  representer->SetLogger(logger);
  auto model = StatisticalModelType::New();
  model = itk::StatismoIO<DataType>::LoadStatisticalModel(representer.GetPointer(), opt.strInputModelFileName);

  StatisticalModelType::Pointer constrainedModel;
  TransformType::Pointer        transform;

  if (opt.strInputFixedLandmarksFileName.empty())
  {
    using BoundingBoxType = itk::BoundingBox<int, gk_dimensions, float, DataType::PointsContainer>;
    // Compute bounding box of model mesh
    auto modelMeshBox = BoundingBoxType::New();
    modelMeshBox->SetPoints(model->GetRepresenter()->GetReference()->GetPoints());
    modelMeshBox->ComputeBoundingBox();

    // Compute bounding box of target mesh
    auto targetMeshBox = BoundingBoxType::New();
    targetMeshBox->SetPoints(targetMesh->GetPoints());
    targetMeshBox->ComputeBoundingBox();

    constrainedModel = model;

    auto modelTransform = StatisticalModelTransformType::New();
    modelTransform->SetStatisticalModel(model);
    modelTransform->SetIdentity();

    // No Landmarks are available: we also have to allow rotation and translation.
    using RotationAndTranslationTransformType = itk::VersorRigid3DTransform<double>;
    auto rotationAndTranslationTransform = RotationAndTranslationTransformType::New();
    rotationAndTranslationTransform->SetIdentity();

    auto center = modelMeshBox->GetCenter();
    rotationAndTranslationTransform->SetCenter(center);

    auto translation = targetMeshBox->GetCenter() - modelMeshBox->GetCenter();
    rotationAndTranslationTransform->SetTranslation(translation);

    using CompositeTransformType = itk::CompositeTransform<double, gk_dimensions>;
    auto compositeTransform = CompositeTransformType::New();
    compositeTransform->AddTransform(rotationAndTranslationTransform);
    compositeTransform->AddTransform(modelTransform);
    compositeTransform->SetAllTransformsToOptimizeOn();

    transform = compositeTransform;
  }
  else
  {
    constrainedModel = statismo::cli::BuildPosteriorShapeModel<DataType, StatisticalModelType, PointsLocatorType>(
      model, opt.strInputFixedLandmarksFileName, opt.strInputMovingLandmarksFileName, opt.dLandmarksVariance, logger);

    StatisticalModelTransformType::Pointer modelTransform = StatisticalModelTransformType::New();
    modelTransform->SetStatisticalModel(constrainedModel);
    modelTransform->SetIdentity();
    transform = modelTransform;
  }

  auto distImage = ComputeDistanceImageForMesh(targetMesh);
  auto optimizer = OptimizerType::New();
  statismo::cli::InitializeOptimizer<OptimizerType>(optimizer,
                                                    opt.uNumberOfIterations,
                                                    model->GetNumberOfPrincipalComponents(),
                                                    transform->GetNumberOfParameters(),
                                                    opt.bPrintFittingInformation,
                                                    logger);


  auto interpolator = InterpolatorType::New();
  auto metric = MetricType::New();
  metric->SetRegularizationParameter(opt.dRegularizationWeight);
  metric->SetNumberOfModelComponents(constrainedModel->GetNumberOfPrincipalComponents());

  DataType::Pointer reference = model->GetRepresenter()->GetReference();

  auto fixedPointSet = PointSetType::New();
  fixedPointSet->SetPoints(reference->GetPoints());

  auto points = PointSetType::PointDataContainer::New();
  points->Reserve(reference->GetNumberOfPoints());
  for (PointSetType::PointDataContainer::Iterator it = points->Begin(); it != points->End(); ++it)
  {
    it->Value() = 0;
  }
  fixedPointSet->SetPointData(points);

  auto registration = RegistrationFilterType::New();
  registration->SetInitialTransformParameters(transform->GetParameters());
  registration->SetMetric(metric);
  registration->SetInterpolator(interpolator);
  registration->SetOptimizer(optimizer);
  registration->SetTransform(transform);
  registration->SetFixedPointSet(fixedPointSet);
  registration->SetMovingImage(distImage);

  pCOSilencer->DisableOutput();
  registration->Update();
  pCOSilencer->EnableOutput();

  auto transformMeshFilter = TransformMeshFilterType::New();
  transformMeshFilter->SetInput(model->GetRepresenter()->GetReference());
  transformMeshFilter->SetTransform(transform);
  transformMeshFilter->Update();

  DataType::Pointer fittedMesh = transformMeshFilter->GetOutput();
  if (!opt.strOutputFittedMeshFileName.empty())
  {
    SaveMesh(fittedMesh, opt.strOutputFittedMeshFileName);
  }

  if (!opt.strOutputProjectedMeshFileName.empty())
  {
    SaveMesh(ProjectOnTargetMesh<PointsLocatorType>(fittedMesh, targetMesh), opt.strOutputProjectedMeshFileName);
  }
}

} // namespace

int
main(int argc, char ** argv)
{
  ProgramOptions                                     poParameters;
  po::program_options<std::string, double, unsigned> parser{ argv[0], "Program help:" };

  parser
    .add_opt<std::string>(
      { "input-model", "i", "The path to the model file.", &poParameters.strInputModelFileName, "" }, true)
    .add_opt<std::string>(
      { "input-targetmesh", "t", "The path to the target mesh.", &poParameters.strInputTargetMeshFileName, "" }, true)
    .add_opt<unsigned>({ "number-of-iterations", "n", "Number of iterations", &poParameters.uNumberOfIterations, 100 },
                       true)
    .add_opt<double>(
      { "regularization-weight",
        "w",
        "This is the regularization weight to make sure the model parameters don't don't get too big while fitting.",
        &poParameters.dRegularizationWeight,
        0.0 },
      true)
    .

    add_opt<std::string>({ "output-fit",
                           "o",
                           "Name of the output file where the fitted mesh will be written to..",
                           &poParameters.strOutputFittedMeshFileName,
                           "" })
    .add_opt<std::string>({ "output-projected",
                            "j",
                            "Name of the output file where the projected mesh will be written to.",
                            &poParameters.strOutputProjectedMeshFileName,
                            "" })
    .

    add_opt<std::string>({ "fixed-landmarks",
                           "f",
                           "Name of the file where the fixed Landmarks are saved.",
                           &poParameters.strInputFixedLandmarksFileName,
                           "" })
    .add_opt<std::string>({ "moving-landmarks",
                            "m",
                            "Name of the file where the moving Landmarks are saved.",
                            &poParameters.strInputMovingLandmarksFileName,
                            "" })
    .add_opt<double>({ "landmarks-variance",
                       "v",
                       "The variance that will be used to build the posterior model.",
                       &poParameters.dLandmarksVariance,
                       1.0f })
    .add_flag(
      { "print-fitting-information",
        "p",
        "Prints information (the parameters, metric score and the iteration count) with each iteration while fitting.",
        &poParameters.bPrintFittingInformation,
        false })
    .add_flag({ "quiet", "q", "Quiet mode (no log).", &poParameters.bIsQuiet, false })
    .add_opt<std::string>({ "log-file", "", "Path to the log file.", &poParameters.strLogFile, "" }, false);


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

  statismo::cli::ConsoleOutputSilencer coSilencer;

  std::unique_ptr<statismo::Logger> logger{ nullptr };
  if (!poParameters.bIsQuiet)
  {
    logger = statismo::cli::CreateLogger(poParameters.strLogFile);
  }

  try
  {
    FitMesh(poParameters, &coSilencer, logger.get());
  }
  catch (const ifstream::failure & e)
  {
    coSilencer.EnableOutput();
    cerr << "Could not read a file:" << endl;
    cerr << e.what() << endl;
    return EXIT_FAILURE;
  }
  catch (itk::ExceptionObject & e)
  {
    coSilencer.EnableOutput();
    cerr << "Could not fit the model:" << endl;
    cerr << e.what() << endl;
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
