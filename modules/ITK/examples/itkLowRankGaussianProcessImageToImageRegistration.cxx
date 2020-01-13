/*
 * This file is part of the statismo library.
 *
 * Author: Christoph Jud (christoph.jud@unibas.ch)
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
 */

#include "statismo/core/Kernels.h"
#include "statismo/core/KernelCombinators.h"
#include "statismo/ITK/itkDataManager.h"
#include "statismo/ITK/itkInterpolatingStatisticalDeformationModelTransform.h"
#include "statismo/ITK/itkLowRankGPModelBuilder.h"
#include "statismo/ITK/itkPosteriorModelBuilder.h"
#include "statismo/ITK/itkStandardImageRepresenter.h"
#include "statismo/ITK/itkStatisticalModel.h"

#include <itkCommand.h>
#include <itkDirectory.h>
#include <itkImage.h>
#include <itkImageFileReader.h>
#include <itkImageFileWriter.h>
#include <itkImageRegistrationMethod.h>
#include <itkLBFGSOptimizer.h>
#include <itkLinearInterpolateImageFunction.h>
#include <itkMeanSquaresImageToImageMetric.h>
#include <itkNormalizedCorrelationImageToImageMetric.h>
#include <itkWarpImageFilter.h>



#include <iostream>
#include <iomanip>
#include <string>

/**
 * This example is to illustrate the full functionality of the Gaussian Process registration using statismo.
 * It can perform a registration of 2D and 3D images. Optionally, landmarks can be provided, which should be matched in
 * the final registration result.
 *
 * Usage:
 *
 * (without landmarks)
 * ./bin/itkLowRankGaussianProcessImageToImageRegistration share/data/hand_images/hand-1.vtk
 * share/data/hand_images/hand-2.vtk /tmp/registered.vtk /tmp/deformationfield.vtk MeanSquares 70 100 100 100
 *
 * (including landmarks)
 * ./bin/itkLowRankGaussianProcessImageToImageRegistration share/data/hand_images/hand-1.vtk
 * share/data/hand_landmarks/hand-1.fcsv share/data/hand_images/hand-2.vtk share/data/hand_landmarks/hand-2.fcsv
 * /tmp/registered.vtk /tmp/deformationfield.vtk MeanSquares 70 100 0.1 100 100
 *
 */

namespace {

//
// Internal type
//

/*
 * Iteration observer of the registration
 */
class _IterationStatusObserver : public itk::Command
{
public:
  using Self = _IterationStatusObserver ;
  using Superclass = itk::Command            ;
  using Pointer = itk::SmartPointer<Self> ;

  itkNewMacro(Self);

  using OptimizerType = itk::LBFGSOptimizer   ;
  using OptimizerPointer = const OptimizerType * ;

  void
  Execute(itk::Object * caller, const itk::EventObject & event) override
  {
    Execute(static_cast<const itk::Object *>(caller), event);
  }

  void
  Execute(const itk::Object * object, const itk::EventObject & event) override
  {
    auto optimizer = dynamic_cast<OptimizerPointer>(object);
    if (!itk::IterationEvent().CheckEvent(&event))
    {
      return;
    }

    std::cout << "Iteration: " << ++m_iterIdx;
    std::cout << "; Value: " << optimizer->GetCachedValue();
    std::cout << "; Current Parameters: " << optimizer->GetCachedCurrentPosition() << std::endl;
  }

private:
  int m_iterIdx{0};
};

/**
 * A scalar valued gaussian kernel.
 */
template <class TPoint>
class _GaussianKernel : public statismo::ScalarValuedKernel<TPoint>
{
public:
  using CoordRepType = typename TPoint::CoordRepType ;
  using VectorType = vnl_vector<CoordRepType>      ;

  explicit _GaussianKernel(double sigma)
    : m_sigma(sigma)
    , m_sigma2(sigma * sigma)
  {}

  inline double
  operator()(const TPoint & x, const TPoint & y) const override
  {
    VectorType xv = x.GetVnlVector();
    VectorType yv = y.GetVnlVector();

    VectorType r = yv - xv;
    return exp(-dot_product(r, r) / m_sigma2);
  }

  std::string
  GetKernelInfo() const override
  {
    std::ostringstream os;
    os << "_GaussianKernel(" << m_sigma << ")";
    return os.str();
  }

private:
  double m_sigma;
  double m_sigma2;
};

//
// Utility routine
//

/*
 * Build a low-rank Gaussian process model using a Gaussian kernel function
 *
 * Input:
 *  - Filename to reference image
 *  - Kernel parameter sigma of Gaussian kernel
 *  - Kernel parameter scale of Gaussian kernel
 *  - Number of basis functions to be approximated
 *
 * Output:
 *  - Smartpointer on a (statismo) statistical model.
 *
 *  - The composition of different kernel functions can be even more complicated. For example
 *    a linear combination of different kernel functions is again a kernel function and thus
 *    can be handled by the LowRankGPModelBuilder.
 */
template <class TRepresenter, class TImage, class TStatisticalModel>
typename TStatisticalModel::Pointer
_BuildLowRankGPModel(const char * referenceFilename,
                    double       gaussianKernelSigma,
                    double       gaussianKernelScale,
                    unsigned     numberOfBasisFunctions)
{

  using ModelBuilderType = itk::LowRankGPModelBuilder<TImage> ;
  using ImageFileReaderType = itk::ImageFileReader<TImage>       ;
  using PointType = typename TImage::PointType         ;

  auto representer = TRepresenter::New();

  std::cout << "Building low-rank Gaussian process deformation model... " << std::flush;

  // we take an arbitrary dataset as the reference, as they have all the same resolution anyway
  auto referenceReader = ImageFileReaderType::New();
  referenceReader->SetFileName(referenceFilename);
  referenceReader->Update();

  representer->SetReference(referenceReader->GetOutput());

  auto gk =
    _GaussianKernel<PointType>(gaussianKernelSigma); // a Gaussian kernel with sigma=gaussianKernelSigma
  // make the kernel matrix valued and scale it by a factor of 100
  const auto& mvGk =
    statismo::UncorrelatedMatrixValuedKernel<PointType>(&gk, representer->GetDimensions());
  const auto& scaledGk =
    statismo::ScaledKernel<PointType>(&mvGk, gaussianKernelScale); // apply Gaussian scale parameter

  auto gpModelBuilder = ModelBuilderType::New();
  gpModelBuilder->SetRepresenter(representer);
auto  model =
    gpModelBuilder->BuildNewZeroMeanModel(scaledGk, numberOfBasisFunctions); // number of basis functions

  std::cout << "done!" << std::endl;
  return model;
}


/*
 * Landmark file reader function.
 *
 * Format for n landmark points
 * x0 y0 z0\n
 * x1 y1 z1\n
 * ...
 * xi yi zi\n
 * ...
 * xn yn zn\n
 *
 * Input:
 *  - std::string landmarkFilename		-> Filename of landmark file.
 *
 * Output:
 *  - std::vector<TPointType>			-> A standard vector of points.
 *
 * Comment:
 * 	- Template parameter TPointType is a statismo StatisticalModelType::PointType e.g. itk::Point<float, 3>
 */
template <class TPointType, unsigned int VImageDimension>
typename std::vector<TPointType>
_ReadLandmarkFile(const std::string& landmarkFilename)
{
  std::vector<TPointType> pointVector;

  const unsigned int x = 0, y = 1, z = 2;
  double             p[3];

  std::ifstream infile{landmarkFilename.c_str()};
  std::string line;
  while (std::getline(infile, line))
  {
    std::stringstream line_stream(line);
    if (!(line_stream >> p[x] && line_stream >> p[y] && line_stream >> p[z]))
    {
      std::stringstream err;
      err << "_ReadLandmarkFile: landmark file is corrupt (filename " << landmarkFilename << ")." << std::endl;
      throw std::runtime_error(err.str());
    }

    TPointType point;
    for (unsigned i = 0; i < VImageDimension; i++)
    {
      point[i] = p[i];
    }
    pointVector.push_back(point);
  }

  return pointVector;
}

/*
 * Build a partially fixed model.
 *
 * The original model is constrainted on the landmarks displacements.
 *
 * Input:
 *  - Original model.
 *  - Filename of the reference landmark file.
 *  - Filename of the target landmark file.
 *  - Landmark uncertainty.
 *
 * Output:
 *  - Smartpointer on the constrainted model.
 *
 * Comment:
 *  - The noise on the landmark measurements is modeled as mean free isotropic Gaussian where
 *    variance parameter is set by  landmarkUncertainty.
 */

template <class TImage, class TStatisticalModel, unsigned int VImageDimension>
typename TStatisticalModel::Pointer
_ConstrainModel(typename TStatisticalModel::Pointer model,
               const std::string &                        referenceLandmarkFilename,
               const std::string &                        targetLandmarkFilename,
               double                              landmarkUncertainty)
{
  using PosteriorModelBuilderType = typename itk::PosteriorModelBuilder<TImage> ;

  typename TStatisticalModel::PointValueListType constraints;

  auto referencePointVector =
    _ReadLandmarkFile<typename TStatisticalModel::PointType, VImageDimension>(referenceLandmarkFilename);
  auto targetPointVector =
    _ReadLandmarkFile<typename TStatisticalModel::PointType, VImageDimension>(targetLandmarkFilename);
  
  assert(referencePointVector.size() != targetPointVector.size());

  for (unsigned i = 0; i < referencePointVector.size(); i++)
  {
    typename TImage::PixelType displacement;
    for (unsigned d = 0; d < VImageDimension; d++)
    {
      displacement[d] = referencePointVector[i][d] - targetPointVector[i][d];
    }

    typename TStatisticalModel::PointValuePairType pointValue(targetPointVector[i], displacement);
    constraints.push_back(pointValue);
  }

  auto pfmb = PosteriorModelBuilderType::New();
  auto         constraintModel =
    pfmb->BuildNewModelFromModel(model.GetPointer(), constraints, landmarkUncertainty);

  return constraintModel;
}

/*
 * Image to image registration method using a statismo statistical model.
 *
 * The standard parametric registration framework of ITK is used for registration, where
 * the transform is a InterpolatingStatisticalDeformationModelTransform.
 *
 * Input:
 *  - Filename of reference image.
 *  - Filename of target image.
 *  - Smartpointer to the statistical model.
 *  - Filename where the resulting deformation field is written ("" = disabled).
 *  - Maximum number of iteration performed by the optimizer.
 *
 * Output:
 *  - TImage::Pointer		-> The registered image (reference image warped by the deformation field).
 */

template <class TImage, class TVectorImage, class TStatisticalModel, class TMetric, unsigned int VImageDimension>
typename TImage::Pointer
_ModelBasedImageToImageRegistration(const std::string&                         referenceFilename,
                                   const std::string&                         targetFilename,
                                   typename TStatisticalModel::Pointer model,
                                   const std::string&                         outputDfFilename,
                                   unsigned                            numberOfIterations)
{


  using ImageReaderType = itk::ImageFileReader<TImage>                                                                  ;
  using TransformType = itk::InterpolatingStatisticalDeformationModelTransform<TVectorImage, double, VImageDimension> ;
  using OptimizerType = itk::LBFGSOptimizer                                                                           ;
  using RegistrationFilterType = itk::ImageRegistrationMethod<TImage, TImage>        ;
  using WarperType = itk::WarpImageFilter<TImage, TImage, TVectorImage>  ;
  using InterpolatorType = itk::LinearInterpolateImageFunction<TImage, double> ;

  auto referenceReader = ImageReaderType::New();
  referenceReader->SetFileName(referenceFilename);
  referenceReader->Update();

  typename TImage::Pointer referenceImage = referenceReader->GetOutput();
  referenceImage->Update();

  auto targetReader = ImageReaderType::New();
  targetReader->SetFileName(targetFilename);
  targetReader->Update();

  typename TImage::Pointer targetImage = targetReader->GetOutput();
  targetImage->Update();

  // do the fitting
  auto transform = TransformType::New();
  transform->SetStatisticalModel(model);
  transform->SetIdentity();

  // Setting up the fitting
  auto optimizer = OptimizerType::New();
  optimizer->MinimizeOn();
  optimizer->SetMaximumNumberOfFunctionEvaluations(numberOfIterations);

  using ObserverType = _IterationStatusObserver;
  auto         observer = ObserverType::New();
  optimizer->AddObserver(itk::IterationEvent(), observer);

  auto        metric = TMetric::New();
  auto interpolator = InterpolatorType::New();

  auto registration = RegistrationFilterType::New();
  registration->SetInitialTransformParameters(transform->GetParameters());
  registration->SetMetric(metric);
  registration->SetOptimizer(optimizer);
  registration->SetTransform(transform);
  registration->SetInterpolator(interpolator);
  registration->SetFixedImage(targetImage);
  registration->SetFixedImageRegion(targetImage->GetBufferedRegion());
  registration->SetMovingImage(referenceImage);

  try
  {
    std::cout << "Performing registration... " << std::flush;
    registration->Update();
    std::cout << "done!" << std::endl;
  }
  catch (itk::ExceptionObject & o)
  {
    std::cerr << "caught exception " << o << std::endl;
  }

  auto df = model->DrawSample(transform->GetCoefficients());

  // write deformation field
  if (!outputDfFilename.empty())
  {
    auto dfWriter = itk::ImageFileWriter<TVectorImage>::New();
    dfWriter->SetFileName(outputDfFilename);
    dfWriter->SetInput(df);
    dfWriter->Update();
  }

  // warp reference
  std::cout << "Warping reference... " << std::flush;
  auto warper = WarperType::New();
  warper->SetInput(referenceImage);
  warper->SetInterpolator(interpolator);
  warper->SetOutputSpacing(targetImage->GetSpacing());
  warper->SetOutputOrigin(targetImage->GetOrigin());
  warper->SetOutputDirection(targetImage->GetDirection());
  warper->SetDisplacementField(df);
  warper->Update();
  std::cout << "done!" << std::endl;

  return warper->GetOutput();
}

/*
 * Performes the image to image registration:
 *  1.	low-rank approximation of the Gaussian process prior
 *  2.	constraining the low-rank model on landmark displacements (if landmarks are defined)
 *  3.	performe the model based image to image registration
 *  4.	write the result on the hdd
 *
 * Input:
 *  referenceFilename			-> Filename of reference image.
 *  referenceLandmarkFilename	-> Filename of reference landmark file ("" = disabled).
 *  targetFilename			-> Filename of target image.
 *  targetLandmarkFilename	-> Filename of target landmark file ("" = disabled).
 *  gaussianKernelSigma			-> Kernel parameter of Gaussian kernel.
 *  gaussianKernelScale			-> Kernel parameter of Gaussian kernel.
 *  landmarkUncertainty			-> Noise parameter to model uncertainty on the landmarks.
 *  similarityMetric			-> Similarity metric for the performance measure in the optimization.
 *  outputFilename			-> Filename of the resulting registration to be written to ("" = disabled).
 *  outputDfFilename			-> Filename of the resulting deformation field to be written to.
 *  numberOfBasisFunctions		-> Number of basis function used in the low-rank approximation.
 *  numberOfIterations			-> Maximum number of iterations to perform in the optimization.
 */
template <class PixelType, unsigned int VImageDimension>
void
RunImageToImageRegistration(const std::string& referenceFilename,
                            const std::string& referenceLandmarkFilename,
                            const std::string& targetFilename,
                            const std::string& targetLandmarkFilename,
                            double      gaussianKernelSigma,
                            double      gaussianKernelScale,
                            double      landmarkUncertainty,
                            const std::string & similarityMetric,
                            const std::string& outputFilename,
                            std::string outputDfFilename,
                            unsigned    numberOfBasisFunctions,
                            unsigned    numberOfIterations)
{
  using ImageType = itk::Image<PixelType, VImageDimension>                                              ;
  using VectorImageType = itk::Image<itk::Vector<float, VImageDimension>, VImageDimension>                    ;
  using RepresenterType = itk::StandardImageRepresenter<typename VectorImageType::PixelType, VImageDimension> ;
  using StatisticalModelType = itk::StatisticalModel<VectorImageType>                                              ;
  using MeanSquaresMetricType = itk::MeanSquaresImageToImageMetric<ImageType, ImageType>                            ;
  using NormalizedCorrelationMetricType = itk::NormalizedCorrelationImageToImageMetric<ImageType, ImageType> ;

  // build deformation model
  auto model =
    _BuildLowRankGPModel<RepresenterType, VectorImageType, StatisticalModelType>(
      referenceFilename.c_str(), gaussianKernelSigma, gaussianKernelScale, numberOfBasisFunctions);
  
  if (!referenceLandmarkFilename.empty() && !targetLandmarkFilename.empty())
  {
    model = _ConstrainModel<VectorImageType, StatisticalModelType, VImageDimension>(
      model, referenceLandmarkFilename, targetLandmarkFilename, landmarkUncertainty);
  }

  // image to image registration with this model
  typename ImageType::Pointer registeredImage;
  if (similarityMetric == "MeanSquares")
  {
    registeredImage = _ModelBasedImageToImageRegistration<ImageType,
                                                         VectorImageType,
                                                         StatisticalModelType,
                                                         MeanSquaresMetricType,
                                                         VImageDimension>(
      referenceFilename, targetFilename, model, outputDfFilename, numberOfIterations);
  }
  else if (similarityMetric == "NormalizedCorrelation")
  {
    registeredImage = _ModelBasedImageToImageRegistration<ImageType,
                                                         VectorImageType,
                                                         StatisticalModelType,
                                                         NormalizedCorrelationMetricType,
                                                         VImageDimension>(
      referenceFilename, targetFilename, model, outputDfFilename, numberOfIterations);
  }

  // write registered image
  if (!outputFilename.empty())
  {
    auto writer = itk::ImageFileWriter<ImageType>::New();
    writer->SetFileName(outputFilename);
    writer->SetInput(registeredImage);
    writer->Update();
  }
}
}


/*
 * Main routine:
 *  1.	parsing parameters (replace this with our favorite options parser)
 *  2.	run registration
 */
int
main(int argc, char * argv[])
{

  // \todo Use lpo here instead of manual parsing

  std::string referenceFilename;
  std::string targetFilename;
  std::string referenceLandmarkFilename;
  std::string targetLandmarkFilename;
  std::string similarityMetric;
  std::string outputFilename;
  std::string outputDfFilename;

  double gaussianKernelSigma = 70;
  double gaussianKernelScale = 100;
  double landmarkUncertainty = 0.1;

  unsigned numberOfBasisFunctions = 100;
  unsigned numberOfIterations = 100;

  // parse command line parameters
  if (argc != 10 && argc != 13)
  {
    std::cout << "***********************************************************" << std::endl;
    std::cout << "usage\t" << argv[0] << std::endl;
    std::cout << "referenceFilename:\t\t Filename of reference image." << std::endl;
    std::cout << "targetFilename:\t\t\t Filename of target image." << std::endl;
    std::cout << "outputFilename:\t\t\t Filename of the resulting registered image." << std::endl;
    std::cout << "outputDfFilename:\t\t Filename of the resulting deformation field." << std::endl;
    std::cout << "similarityMetric:\t\t Similarity metric: MeanSquares and NormalizedCorrelation are supported."
              << std::endl;
    std::cout << "gaussianKernelSigma:\t\t Sigma of the Gaussian kernel. (e.g. 70)" << std::endl;
    std::cout << "gaussianKernelScale:\t\t Scale of the Gaussian kernel. (e.g. 100)" << std::endl;
    std::cout << "numBasisFunctions:\t\t Number of basis functions to approximate. (e.g. 100)" << std::endl;
    std::cout << "numIterations:\t\t\t Number of iterations to perform in the optimization. (e.g. 100)" << std::endl
              << std::endl;

    std::cout << "or (including landmarks)\t" << argv[0] << std::endl;
    std::cout << "referenceFilename:\t\t Filename of reference image." << std::endl;
    std::cout << "referenceLandmarkFilename:\t Filename of reference landmark file." << std::endl;
    std::cout << "targetFilename:\t\t\t Filename of target image." << std::endl;
    std::cout << "targetLandmarkFilename:\t\t Filename of target landmark file." << std::endl;
    std::cout << "outputFilename:\t\t\t Filename of the resulting registered image." << std::endl;
    std::cout << "outputDfFilename:\t\t Filename of the resulting deformation field." << std::endl;
    std::cout << "similarityMetric:\t\t Similarity metric: MeanSquares and NormalizedCorrelation are supported."
              << std::endl;
    std::cout << "gaussianKernelSigma:\t\t Sigma of the Gaussian kernel. (e.g. 70)" << std::endl;
    std::cout << "gaussianKernelScale:\t\t Scale of the Gaussian kernel. (e.g. 100)" << std::endl;
    std::cout << "landmarkUncertainty:\t\t Uncertainty of the landmarks. (e.g. 0.1)" << std::endl;
    std::cout << "numBasisFunctions:\t\t Number of basis functions to approximate. (e.g. 100)" << std::endl;
    std::cout << "numIterations:\t\t\t Number of iterations to perform in the optimization. (e.g. 100)" << std::endl;
    return 1;
  }

  if (argc == 10)
  {
    referenceFilename = argv[1];
    targetFilename = argv[2];
    outputFilename = argv[3];
    outputDfFilename = argv[4];
    similarityMetric = argv[5];

    std::stringstream ss;
    ss << argv[6];
    ss >> gaussianKernelSigma;
    ss.str("");
    ss.clear();
    ss << argv[7];
    ss >> gaussianKernelScale;
    ss.str("");
    ss.clear();
    ss << argv[8];
    ss >> numberOfBasisFunctions;
    ss.str("");
    ss.clear();
    ss << argv[9];
    ss >> numberOfIterations;
  }

  if (argc == 13)
  {
    referenceFilename = argv[1];
    referenceLandmarkFilename = argv[2];
    targetFilename = argv[3];
    targetLandmarkFilename = argv[4];
    outputFilename = argv[5];
    outputDfFilename = argv[6];
    similarityMetric = argv[7];

    std::stringstream ss;
    ss << argv[8];
    ss >> gaussianKernelSigma;
    ss.str("");
    ss.clear();
    ss << argv[9];
    ss >> gaussianKernelScale;
    ss.str("");
    ss.clear();
    ss << argv[10];
    ss >> landmarkUncertainty;
    ss.str("");
    ss.clear();
    ss << argv[11];
    ss >> numberOfBasisFunctions;
    ss.str("");
    ss.clear();
    ss << argv[12];
    ss >> numberOfIterations;
  }

  // derive number of space dimensions
  itk::ImageIOBase::Pointer imageIO =
    itk::ImageIOFactory::CreateImageIO(referenceFilename.c_str(), itk::ImageIOFactory::ReadMode);

  imageIO->SetFileName(referenceFilename);
  imageIO->ReadImageInformation();
  const unsigned numDimensions = imageIO->GetNumberOfDimensions();


  // print out parameters
  std::cout << "************************************************" << std::endl;
  std::cout << "Low-rank Gaussian process image registration:" << std::endl;
  std::cout << " - space dimensions\t\t" << numDimensions << std::endl;
  std::cout << " - reference\t\t\t" << referenceFilename << std::endl;
  if (!referenceLandmarkFilename.empty()) {
    std::cout << " - reference landmarks\t\t" << referenceLandmarkFilename << std::endl;
  }
  std::cout << " - target\t\t\t" << targetFilename << std::endl;
  if (!targetLandmarkFilename.empty()) {
    std::cout << " - target landmarks\t\t" << targetLandmarkFilename << std::endl;
  }
  std::cout << " - output\t\t\t" << outputFilename << std::endl;
  std::cout << " - output deformation field\t" << outputDfFilename << std::endl << std::endl;
  std::cout << " - similarity metric\t\t" << similarityMetric << std::endl;
  std::cout << " - Gaussian sigma\t\t" << gaussianKernelSigma << std::endl;
  std::cout << " - Gaussian scale\t\t" << gaussianKernelScale << std::endl;
  if (!targetLandmarkFilename.empty() && !referenceLandmarkFilename.empty()) {
    std::cout << " - Landmark uncertainty\t\t" << landmarkUncertainty << std::endl;
  }
  std::cout << " - #basis functions\t\t" << numberOfBasisFunctions << std::endl;
  std::cout << " - #iterations\t\t\t" << numberOfIterations << std::endl << std::endl;


  if (!(similarityMetric == "NormalizedCorrelation" || similarityMetric == "MeanSquares"))
  {
    std::cerr << "Error: only MeanSquares or NormalizedCorrelation supported as metric." << std::endl;
    return 1;
  }

  if (landmarkUncertainty == 0)
  {
    std::cout << "Warning: landmark uncertainty sould be greater than zero." << std::endl;
  }

  if (landmarkUncertainty < 0)
  {
    std::cerr << "Error: landmark uncertainty has to be positive." << std::endl;
    return 1;
  }

  if (numDimensions == 2)
  { // run the image to image registration in 2D
    RunImageToImageRegistration<float, 2>(referenceFilename,
                                          referenceLandmarkFilename,
                                          targetFilename,
                                          targetLandmarkFilename,
                                          gaussianKernelSigma,
                                          gaussianKernelScale,
                                          landmarkUncertainty,
                                          similarityMetric,
                                          outputFilename,
                                          outputDfFilename,
                                          numberOfBasisFunctions,
                                          numberOfIterations);
  }
  else if (numDimensions == 3)
  { // run the image to image registration in 2D
    RunImageToImageRegistration<float, 3>(referenceFilename,
                                          referenceLandmarkFilename,
                                          targetFilename,
                                          targetLandmarkFilename,
                                          gaussianKernelSigma,
                                          gaussianKernelScale,
                                          landmarkUncertainty,
                                          similarityMetric,
                                          outputFilename,
                                          outputDfFilename,
                                          numberOfBasisFunctions,
                                          numberOfIterations);
  } else {
    std::cerr << "expected dimension 2 or 3" << std::endl;
    return 1;
  }

  std::cout << "Low-rank Gaussian process image to image registration completed successfully." << std::endl;
  return 0;
}
