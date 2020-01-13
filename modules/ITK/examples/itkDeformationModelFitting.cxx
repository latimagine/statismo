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

/*
 * This example shows how the fitting of a statistical deformation model can be performed with statismo.
 */

#include "statismo/ITK/itkInterpolatingStatisticalDeformationModelTransform.h"
#include "statismo/ITK/itkStandardImageRepresenter.h"
#include "statismo/ITK/itkStatisticalModel.h"
#include "statismo/ITK/itkIO.h"

#include <itkCommand.h>
#include <itkImageFileReader.h>
#include <itkImageRegistrationMethod.h>
#include <itkLBFGSOptimizer.h>
#include <itkLinearInterpolateImageFunction.h>
#include <itkMeanSquaresImageToImageMetric.h>
#include <itkNormalizedCorrelationImageToImageMetric.h>

namespace {

const unsigned                                                                               Dimensions = 2;
using ImageType = itk::Image<uint16_t, Dimensions>                                           ;
using VectorImageType = itk::Image<itk::Vector<float, ImageType::ImageDimension>, ImageType::ImageDimension> ;
using RepresenterType = itk::StandardImageRepresenter<itk::Vector<float, Dimensions>, Dimensions> ;
using ImageReaderType = itk::ImageFileReader<ImageType> ;
// using = itk::MeanSquaresImageToImageMetric<ImageType, ImageType> MetricType;
using MetricType = itk::NormalizedCorrelationImageToImageMetric<ImageType, ImageType>                          ;
using TransformType = itk::InterpolatingStatisticalDeformationModelTransform<VectorImageType, double, Dimensions> ;
using InterpolatorType = itk::LinearInterpolateImageFunction<ImageType, double>                                      ;
using RegistrationFilterType = itk::ImageRegistrationMethod<ImageType, ImageType> ;
using OptimizerType = itk::LBFGSOptimizer ;
using StatisticalModelType = itk::StatisticalModel<VectorImageType> ;


class _IterationStatusObserver : public itk::Command
{
public:
  using Self = _IterationStatusObserver ;
  using Superclass = itk::Command            ;
  using Pointer = itk::SmartPointer<Self> ;

  itkNewMacro(Self);

  using OptimizerType = itk::LBFGSOptimizer ;
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

}

/*
 * The fixedImage needs to correspond to the image that was used to obtain the displacement fields of the model
 * (e.g. the fixed image in the registration that generated the displacement fields).
 */
int
main(int argc, char * argv[])
{
  if (argc < 5)
  {
    std::cout << "usage " << argv[0] << " modelname fixedImage movingImage output-df" << std::endl;
    exit(-1);
  }

  const char * modelname = argv[1];
  const char * referencename = argv[2];
  const char * targetname = argv[3];
  const char * outdfname = argv[4];

  auto refReader = ImageReaderType::New();
  refReader->SetFileName(referencename);
  refReader->Update();
  ImageType::Pointer refImage = refReader->GetOutput();

  auto targetReader = ImageReaderType::New();
  targetReader->SetFileName(targetname);
  targetReader->Update();
  ImageType::Pointer targetImage = targetReader->GetOutput();

  auto    representer = RepresenterType::New();
  auto model = StatisticalModelType::New();
  model = itk::StatismoIO<VectorImageType>::LoadStatisticalModel(representer, modelname);

  // do the fitting
  auto transform = TransformType::New();
  transform->SetStatisticalModel(model);
  transform->SetIdentity();

  // Setting up the fitting
  auto optimizer = OptimizerType::New();
  optimizer->MinimizeOn();
  optimizer->SetMaximumNumberOfFunctionEvaluations(100);

  using ObserverType = _IterationStatusObserver;
  auto        observer = ObserverType::New();
  optimizer->AddObserver(itk::IterationEvent(), observer);

  auto     metric = MetricType::New();
  auto interpolator = InterpolatorType::New();

  auto registration = RegistrationFilterType::New();
  registration->SetInitialTransformParameters(transform->GetParameters());
  registration->SetMetric(metric);
  registration->SetOptimizer(optimizer);
  registration->SetTransform(transform);
  registration->SetInterpolator(interpolator);
  registration->SetFixedImage(refImage);
  registration->SetFixedImageRegion(refImage->GetBufferedRegion()); // seems to be necessary for the filter to work
  registration->SetMovingImage(targetImage);

  try
  {
    registration->Update();
  }
  catch (itk::ExceptionObject & o)
  {
    std::cerr << "failed with exception " << o << std::endl;
    return 1;
  }

  auto df = model->DrawSample(transform->GetCoefficients());

  auto writer = itk::ImageFileWriter<VectorImageType>::New();
  writer->SetFileName(outdfname);
  writer->SetInput(df);
  writer->Update();

  return 0;
}
