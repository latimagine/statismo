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
#include "utils/itkPenalizingMeanSquaresImageToImageMetric.h"
#include "utils/statismoFittingUtils.h"
#include "utils/statismoBuildPosteriorModelUtils.h"

#include "statismo/ITK/itkInterpolatingStatisticalDeformationModelTransform.h"
#include "statismo/ITK/itkStandardImageRepresenter.h"
#include "statismo/ITK/itkIO.h"
#include "statismo/ITK/itkStatisticalModel.h"

#include "lpo.h"

#include <itkCommand.h>
#include <itkCompositeTransform.h>
#include <itkImageFileReader.h>
#include <itkImageRegistrationMethod.h>
#include <itkLBFGSOptimizer.h>
#include <itkRegularStepGradientDescentOptimizer.h>
#include <itkLinearInterpolateImageFunction.h>
#include <itkNormalizedCorrelationImageToImageMetric.h>
#include <itkRigid2DTransform.h>
#include <itkTransformToDisplacementFieldFilter.h>
#include <itkVersorRigid3DTransform.h>
#include <itkWarpImageFilter.h>
#include <itkCenteredTransformInitializer.h>

namespace po = lpo;
using namespace std;

namespace
{

constexpr unsigned gk_dimensionality2D = 2;
constexpr unsigned gk_dimensionality3D = 3;

struct ProgramOptions
{
  string strInputModelFileName;
  string strInputMovingImageFileName;
  string strInputFixedImageFileName;

  string strOutputFittedImageFileName;
  string strOutputModelTransformFileName;
  string strOutputEntireTransformFileName;

  string strInputFixedLandmarksFileName;
  string strInputMovingLandmarksFileName;
  double dLandmarksVariance{ 0.0 };

  unsigned uNumberOfDIMENSIONS{ 0 };
  unsigned uNumberOfIterations{ 0 };
  double   dRegularizationWeight{ 0.0 };

  bool bPrintFittingInformation{ false };

  bool        bIsQuiet{ false };
  std::string strLogFile{ "" };
};

bool
IsOptionsConflictPresent(const ProgramOptions & opt)
{
  return ((!opt.strInputFixedLandmarksFileName.empty()) ^ (!opt.strInputMovingLandmarksFileName.empty())) ||
         opt.strInputFixedImageFileName.empty() || opt.strInputModelFileName.empty() ||
         opt.strInputMovingImageFileName.empty() ||
         (opt.strOutputEntireTransformFileName.empty() && opt.strOutputFittedImageFileName.empty() &&
          opt.strOutputModelTransformFileName.empty());
}

template <class ImageType>
void
SaveImage(typename ImageType::Pointer img, const std::string & outputFileName)
{
  using ImageWriter = itk::ImageFileWriter<ImageType>;
  auto writer = ImageWriter::New();
  writer->SetInput(img);
  writer->SetFileName(outputFileName);
  writer->Update();
}

template <class DisplacementFieldImageType, class ReferenceImageType, class TransformType>
typename DisplacementFieldImageType::Pointer
GenerateAndSaveDisplacementField(typename ReferenceImageType::Pointer refImg,
                                 typename TransformType::Pointer      tf,
                                 const std::string &                  outputFileName)
{
  using DisplacementFieldGeneratorType = itk::TransformToDisplacementFieldFilter<DisplacementFieldImageType, double>;
  auto fieldGen = DisplacementFieldGeneratorType::New();
  fieldGen->UseReferenceImageOn();
  fieldGen->SetReferenceImage(refImg);
  fieldGen->SetTransform(tf);
  fieldGen->Update();

  typename DisplacementFieldImageType::Pointer dispField = fieldGen->GetOutput();

  if (!outputFileName.empty())
  {
    SaveImage<DisplacementFieldImageType>(dispField, outputFileName);
  }

  return dispField;
}

template <unsigned DIMENSIONS, typename RotationAndTranslationTransformType>
void
FitImage(const ProgramOptions & opt, statismo::cli::ConsoleOutputSilencer * coSilencer, statismo::Logger * logger)
{
  using ImageType = itk::Image<float, DIMENSIONS>;
  using ImageReaderType = itk::ImageFileReader<ImageType>;
  using VectorPixelType = itk::Vector<float, DIMENSIONS>;
  using VectorImageType = itk::Image<VectorPixelType, DIMENSIONS>;
  using RepresenterType = itk::StandardImageRepresenter<VectorPixelType, DIMENSIONS>;
  using TransformInitializerType =
    itk::CenteredTransformInitializer<RotationAndTranslationTransformType, ImageType, ImageType>;
  using CompositeTransformType = itk::CompositeTransform<double, DIMENSIONS>;
  using StatisticalModelType = itk::StatisticalModel<VectorImageType>;
  using TransformType = itk::Transform<double, DIMENSIONS, DIMENSIONS>;
  using ModelTransformType =
    itk::InterpolatingStatisticalDeformationModelTransform<VectorImageType, double, DIMENSIONS>;
  using OptimizerType = itk::LBFGSOptimizer;
  using MetricType = itk::PenalizingMeanSquaresImageToImageMetric<ImageType, ImageType>;
  using InterpolatorType = itk::LinearInterpolateImageFunction<ImageType, double>;
  using RegistrationFilterType = itk::ImageRegistrationMethod<ImageType, ImageType>;

  auto fixedImageReader = ImageReaderType::New();
  fixedImageReader->SetFileName(opt.strInputFixedImageFileName);
  fixedImageReader->Update();
  typename ImageType::Pointer fixedImage = fixedImageReader->GetOutput();

  auto movingImageReader = ImageReaderType::New();
  movingImageReader->SetFileName(opt.strInputMovingImageFileName);
  movingImageReader->Update();
  typename ImageType::Pointer movingImage = movingImageReader->GetOutput();

  auto representer = RepresenterType::New();
  representer->SetLogger(logger);
  auto model = itk::StatismoIO<VectorImageType>::LoadStatisticalModel(representer, opt.strInputModelFileName);

  typename TransformType::Pointer transform;
  auto                            modelTransform = ModelTransformType::New();
  if (!opt.strInputMovingLandmarksFileName.empty())
  {
    model = statismo::cli::BuildPosteriorDeformationModel<VectorImageType, StatisticalModelType>(
      model, opt.strInputFixedLandmarksFileName, opt.strInputMovingLandmarksFileName, opt.dLandmarksVariance, logger);
    transform = modelTransform;
  }
  else
  {
    // No Landmarks are available: we also have to allow rotation and translation.
    auto rotationAndTranslationTransform = RotationAndTranslationTransformType::New();
    rotationAndTranslationTransform->SetIdentity();

    auto initializer = TransformInitializerType::New();
    initializer->SetTransform(rotationAndTranslationTransform);
    initializer->SetFixedImage(fixedImage);
    initializer->SetMovingImage(movingImage);
    initializer->MomentsOn();
    initializer->InitializeTransform();

    auto compositeTransform = CompositeTransformType::New();
    compositeTransform->AddTransform(rotationAndTranslationTransform);
    compositeTransform->AddTransform(modelTransform);
    compositeTransform->SetAllTransformsToOptimizeOn();

    transform = compositeTransform;
  }

  modelTransform->SetStatisticalModel(model);
  modelTransform->SetIdentity();

  auto optimizer = OptimizerType::New();
  statismo::cli::InitializeOptimizer<OptimizerType>(optimizer,
                                                    opt.uNumberOfIterations,
                                                    model->GetNumberOfPrincipalComponents(),
                                                    transform->GetNumberOfParameters(),
                                                    opt.bPrintFittingInformation,
                                                    logger);


  auto metric = MetricType::New();
  metric->SetRegularizationParameter(opt.dRegularizationWeight);
  metric->SetNumberOfModelComponents(model->GetNumberOfPrincipalComponents());

  auto interpolator = InterpolatorType::New();
  auto registration = RegistrationFilterType::New();
  registration->SetInitialTransformParameters(transform->GetParameters());
  registration->SetMetric(metric);
  registration->SetOptimizer(optimizer);
  registration->SetTransform(transform);
  registration->SetInterpolator(interpolator);
  registration->SetFixedImage(fixedImage);
  registration->SetFixedImageRegion(fixedImage->GetBufferedRegion());
  registration->SetMovingImage(movingImage);

  coSilencer->DisableOutput();
  registration->Update();
  coSilencer->EnableOutput();

  auto displacementField = GenerateAndSaveDisplacementField<VectorImageType, ImageType, TransformType>(
    fixedImage, transform, opt.strOutputEntireTransformFileName);

  GenerateAndSaveDisplacementField<VectorImageType, ImageType, TransformType>(
    fixedImage, static_cast<typename TransformType::Pointer>(modelTransform), opt.strOutputModelTransformFileName);

  if (!opt.strOutputFittedImageFileName.empty())
  {
    using WarpFilterType = itk::WarpImageFilter<ImageType, ImageType, VectorImageType>;
    auto warper = WarpFilterType::New();
    warper->SetInput(movingImage);
    warper->SetInterpolator(interpolator);
    warper->SetOutputSpacing(fixedImage->GetSpacing());
    warper->SetOutputOrigin(fixedImage->GetOrigin());
    warper->SetOutputDirection(fixedImage->GetDirection());
    warper->SetDisplacementField(displacementField);
    warper->Update();

    SaveImage<ImageType>(warper->GetOutput(), opt.strOutputFittedImageFileName);
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
      { "moving-image", "m", "The path to the moving image.", &poParameters.strInputMovingImageFileName, "" }, true)
    .add_opt<std::string>(
      { "fixed-image", "f", "The path to the fixed image.", &poParameters.strInputFixedImageFileName, "" }, true)
    .add_opt<unsigned>({ "dimensionality",
                         "d",
                         "Dimensionality of the input image and model",
                         &poParameters.uNumberOfDIMENSIONS,
                         3,
                         2,
                         3 },
                       true)
    .add_opt<unsigned>({ "number-of-iterations", "n", "Number of iterations", &poParameters.uNumberOfIterations, 100 },
                       true)
    .add_opt<double>(
      { "regularization-weight",
        "w",
        "This is the regularization weight to make sure the model parameters don't don't get too big while fitting.",
        &poParameters.dRegularizationWeight,
        0.0 },
      true)
    .add_opt<std::string>({ "output-model-deformationfield",
                            "a",
                            "Name of the output file where the model deformation field will be written to.",
                            &poParameters.strOutputModelTransformFileName,
                            "" })
    .add_opt<std::string>({ "output-deformationfield",
                            "e",
                            "Name of the output file where the entire deformation field will be written to. This "
                            "includes the rotation and translation (Only use this when NOT using landmarks).",
                            &poParameters.strOutputEntireTransformFileName,
                            "" })
    .add_opt<std::string>({ "fixed-landmarks",
                            "",
                            "Name of the file where the fixed Landmarks are saved.",
                            &poParameters.strInputFixedLandmarksFileName,
                            "" })
    .add_opt<std::string>({ "moving-landmarks",
                            "",
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
    .add_pos_opt<std::string>({ "Name of the output file where the fitted image will be written to.",
                                &poParameters.strOutputFittedImageFileName })
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
  std::unique_ptr<statismo::Logger>    logger{ nullptr };
  if (!poParameters.bIsQuiet)
  {
    logger = statismo::cli::CreateLogger(poParameters.strLogFile);
  }

  try
  {
    if (poParameters.uNumberOfDIMENSIONS == gk_dimensionality2D)
    {
      using RotationAndTranslationTransformType = itk::Rigid2DTransform<double>;
      FitImage<gk_dimensionality2D, RotationAndTranslationTransformType>(poParameters, &coSilencer, logger.get());
    }
    else
    {
      using RotationAndTranslationTransformType = itk::VersorRigid3DTransform<double>;
      FitImage<gk_dimensionality3D, RotationAndTranslationTransformType>(poParameters, &coSilencer, logger.get());
    }
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
