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
 * This example shows how the fitting of a statistical shape model to a Mesh can be performed with Statismo and the itk
 * Registration framework.
 */

#include "statismo/ITK/itkStandardMeshRepresenter.h"
#include "statismo/ITK/itkIO.h"
#include "statismo/ITK/itkStatisticalModel.h"
#include "statismo/ITK/itkStatisticalShapeModelTransform.h"

#include <itkCommand.h>
#include <itkEuclideanDistancePointMetric.h>
#include <itkLevenbergMarquardtOptimizer.h>
#include <itkMesh.h>
#include <itkMeshFileReader.h>
#include <itkPointSetToPointSetRegistrationMethod.h>

namespace
{
constexpr unsigned gk_dimensions = 3;
using MeshType = itk::Mesh<float, gk_dimensions>;
using RepresenterType = itk::StandardMeshRepresenter<float, gk_dimensions>;
using MeshReaderType = itk::MeshFileReader<MeshType>;
using MetricType = itk::EuclideanDistancePointMetric<MeshType, MeshType>;
// As a transform, we use the StatisticalShapeModelTransform, that comes with statismo
using TransformType = itk::StatisticalShapeModelTransform<MeshType, double, gk_dimensions>;
using RegistrationFilterType = itk::PointSetToPointSetRegistrationMethod<MeshType, MeshType>;
using OptimizerType = itk::LevenbergMarquardtOptimizer;
using StatisticalModelType = itk::StatisticalModel<MeshType>;

class IterationStatusObserver : public itk::Command
{
public:
  using Self = IterationStatusObserver;
  using Superclass = itk::Command;
  using Pointer = itk::SmartPointer<Self>;

  itkNewMacro(Self);

  using OptimizerPointer = const OptimizerType *;

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

    std::cout << "Iteration: " << ++m_iterdx << " model arameters " << optimizer->GetCachedCurrentPosition()
              << std::endl;
  }

private:
  int m_iterdx{ 0 };
};

} // namespace

int
main(int argc, char * argv[])
{
  if (argc < 4)
  {
    std::cerr << "usage " << argv[0] << " modelname target outputmesh" << std::endl;
    return 1;
  }

  const char * modelname = argv[1];
  const char * targetname = argv[2];
  const char * outputmeshname = argv[3];

  // load the model
  auto              representer = RepresenterType::New();
  auto              model = itk::StatismoIO<MeshType>::LoadStatisticalModel(representer, modelname);
  MeshType::Pointer fixedPointSet = model->GetRepresenter()->GetReference();
  std::cout << "model succesully loaded " << std::endl;

  // load the image to which we will fit
  auto targetReader = MeshReaderType::New();
  targetReader->SetFileName(targetname);
  targetReader->Update();
  MeshType::Pointer targetMesh = targetReader->GetOutput();


  // now we perform the fitting, using the itk registration framework
  auto transform = TransformType::New();
  transform->SetStatisticalModel(model);
  transform->SetIdentity();

  // Setting up the fitting
  auto optimizer = OptimizerType::New();
  optimizer->SetNumberOfIterations(100);
  optimizer->SetUseCostFunctionGradient(false);
  optimizer->SetGradientTolerance(1e-5);
  optimizer->SetValueTolerance(1e-5);
  optimizer->SetEpsilonFunction(1e-6);

  using ObserverType = IterationStatusObserver;
  auto observer = ObserverType::New();
  optimizer->AddObserver(itk::IterationEvent(), observer);

  auto metric = MetricType::New();

  auto registration = RegistrationFilterType::New();
  registration->SetInitialTransformParameters(transform->GetParameters());
  registration->SetMetric(metric);
  registration->SetOptimizer(optimizer);
  registration->SetTransform(transform);
  registration->SetFixedPointSet(targetMesh);
  registration->SetMovingPointSet(fixedPointSet);

  try
  {
    std::cout << "starting model fitting" << std::endl;
    registration->Update();
  }
  catch (itk::ExceptionObject & o)
  {
    std::cerr << "caught exception " << o << std::endl;
    return 1;
  }

  // We obtain the fitting result by drawing the model instance that belongs to the
  // optimal tranform parameters (coefficients)
  auto mesh = model->DrawSample(transform->GetCoefficients());

  // Write out the fitting result
  auto writer = itk::MeshFileWriter<MeshType>::New();
  writer->SetFileName(outputmeshname);
  writer->SetInput(mesh);
  writer->Update();

  return 0;
}
