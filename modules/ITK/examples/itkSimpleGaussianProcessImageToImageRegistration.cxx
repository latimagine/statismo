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
#include "statismo/ITK/itkStandardImageRepresenter.h"
#include "statismo/ITK/itkStatisticalModel.h"

#include "itkCommand.h"
#include "itkDirectory.h"
#include "itkImage.h"
#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"
#include "itkImageRegistrationMethod.h"
#include "itkLBFGSOptimizer.h"
#include "itkLinearInterpolateImageFunction.h"
#include "itkMeanSquaresImageToImageMetric.h"


#include <iostream>
#include <iomanip>
#include <string>

/**
 * This example is to illustrate basic functionality of the Gaussian Process registration using statismo.
 *
 * Usage:
 * ./bin/itkSimpleGaussianProcessImageToImageRegistration share/data/hand_images/hand-1.vtk
 * share/data/hand_images/hand-2.vtk /tmp/deformationfield.vtk
 */

namespace
{

constexpr int gk_gaussianSigma = 70;
constexpr int gk_gaussianScale = 100;
constexpr int gk_numBasisFunctions = 100;
constexpr int gk_numIterations = 100;

/**
 * A scalar valued gaussian kernel.
 */
template <class TPoint>
class GaussianKernel : public statismo::ScalarValuedKernel<TPoint>
{
public:
  using CoordRepType = typename TPoint::CoordRepType;
  using VectorType = vnl_vector<CoordRepType>;

  explicit GaussianKernel(double sigma)
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
    os << "GaussianKernel(" << m_sigma << ")";
    return os.str();
  }

private:
  double m_sigma;
  double m_sigma2;
};


/*
 * Build a low-rank Gaussian process model using a Gaussian kernel function
 *
 * Input:
 *  - const char* referenceFilename		-> Filename to reference image
 *
 * Output:
 *  - StatisticalModelType::Pointer	-> Smartpointer on a (statismo) statistical model.
 *
 * Comment:
 *  - The composition of different kernel functions can be even more complicated. For example
 *    a linear combination of different kernel functions is again a kernel function and thus
 *    can be handled by the LowRankGPModelBuilder.
 */
template <typename RepresenterType, typename Image, typename StatisticalModelType>
typename StatisticalModelType::Pointer
BuildLowRankGPModel(const char * referenceFilename)
{
  using ModelBuilderType = itk::LowRankGPModelBuilder<Image>;
  using ImageFileReaderType = itk::ImageFileReader<Image>;
  using PointType = typename RepresenterType::PointType;

  // we take an arbitrary dataset as the reference, as they have all the same resolution anyway
  auto referenceReader = ImageFileReaderType::New();
  referenceReader->SetFileName(referenceFilename);
  referenceReader->Update();

  auto representer = RepresenterType::New();
  representer->SetReference(referenceReader->GetOutput());

  auto gk = GaussianKernel<PointType>(gk_gaussianSigma); // a Gaussian kernel with sigma=gaussianKernelSigma
  // make the kernel matrix valued and scale it by a factor of 100
  const auto & mvGk = statismo::UncorrelatedMatrixValuedKernel<PointType>(&gk, representer->GetDimensions());
  const auto & scaledGk = statismo::ScaledKernel<PointType>(&mvGk, gk_gaussianScale); // apply Gaussian scale parameter

  auto gpModelBuilder = ModelBuilderType::New();
  gpModelBuilder->SetRepresenter(representer);
  auto model = gpModelBuilder->BuildNewZeroMeanModel(scaledGk, gk_numBasisFunctions); // number of basis functions

  std::cout << "done!" << std::endl;
  return model;
}

/*
 * Image to image registration method using a statismo statistical model.
 *
 * The standard parametric registration framework of ITK is used for registration, where
 * the transform is a InterpolatingStatisticalDeformationModelTransform.
 *
 * Input:
 *  - std::string referenceFilename			-> Filename of reference image.
 *  - std::string targetFilename			-> Filename of target image.
 *  - StatisticalModelType::Pointer model	-> Smartpointer to the statistical model.
 *
 * Output:
 *  - VectorImage::Pointer		-> The deformation field.
 */
template <typename Image, typename VectorImage, typename StatisticalModelType, typename Metric, unsigned int IMAGE_DIM>
typename VectorImage::Pointer
ModelBasedImageToImageRegistration(const std::string &                     referenceFilename,
                                   const std::string &                     targetFilename,
                                   typename StatisticalModelType::Pointer model)
{

  using ImageReaderType = itk::ImageFileReader<Image>;
  using TransformType = itk::InterpolatingStatisticalDeformationModelTransform<VectorImage, double, IMAGE_DIM>;
  using OptimizerType = itk::LBFGSOptimizer;
  using RegistrationFilterType = itk::ImageRegistrationMethod<Image, Image>;
  using InterpolatorType = itk::LinearInterpolateImageFunction<Image, double>;

  auto referenceReader = ImageReaderType::New();
  referenceReader->SetFileName(referenceFilename);
  referenceReader->Update();
  typename Image::Pointer referenceImage = referenceReader->GetOutput();

  auto targetReader = ImageReaderType::New();
  targetReader->SetFileName(targetFilename);
  targetReader->Update();
  typename Image::Pointer targetImage = targetReader->GetOutput();

  // do the fitting
  auto transform = TransformType::New();
  transform->SetStatisticalModel(model);
  transform->SetIdentity();

  // Setting up the fitting
  auto optimizer = OptimizerType::New();
  optimizer->MinimizeOn();
  optimizer->SetMaximumNumberOfFunctionEvaluations(gk_numIterations);

  auto metric = Metric::New();
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
    return nullptr;
  }

  return model->DrawSample(transform->GetCoefficients());
}
} // namespace

/*
 * Main routine:
 */
int
main(int argc, char * argv[])
{
  // parse command line parameters
  if (argc != 4)
  {
    std::cout << "usage\t" << argv[0] << " referenceFilename targetFilename outputFilename" << std::endl;
    exit(-1);
  }

  std::string referenceFilename = argv[1];
  std::string targetFilename = argv[2];
  std::string outputFilename = argv[3];

  using ImageType = itk::Image<float, 2>;
  using VectorImageType = itk::Image<itk::Vector<float, 2>, 2>;
  using RepresenterType = itk::StandardImageRepresenter<VectorImageType::PixelType, 2>;
  using StatisticalModelType = itk::StatisticalModel<VectorImageType>;
  using MeanSquaresMetricType = itk::MeanSquaresImageToImageMetric<ImageType, ImageType>;

  // perform low-rank approximation of Gaussian process prior
  auto model = BuildLowRankGPModel<RepresenterType, VectorImageType, StatisticalModelType>(referenceFilename.c_str());

  // perform image to image registration using the Gaussian process deformation model
  auto deformationField =
    ModelBasedImageToImageRegistration<ImageType, VectorImageType, StatisticalModelType, MeanSquaresMetricType, 2>(
      referenceFilename, targetFilename, model);

  // write deformation field
  auto dfWriter = itk::ImageFileWriter<VectorImageType>::New();
  dfWriter->SetFileName(outputFilename);
  dfWriter->SetInput(deformationField);
  dfWriter->Update();

  std::cout << "Low-rank Gaussian process image to image registration has been successfully finished." << std::endl;
  return 0;
}
