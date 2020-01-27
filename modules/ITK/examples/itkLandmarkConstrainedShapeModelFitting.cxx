/*
 * This file is part of the statismo library.
 *
 * Author: Marcel Luethi (marcel.luethi@unibas.ch)
 *         Ghazi Bouabene (ghazi.bouabene@unibas.ch)
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
#include "statismo/ITK/itkPosteriorModelBuilder.h"
#include "statismo/ITK/itkStandardMeshRepresenter.h"
#include "statismo/ITK/itkIO.h"
#include "statismo/ITK/itkStatisticalModel.h"
#include "statismo/ITK/itkStatisticalShapeModelTransform.h"

#include <itkBinaryThresholdImageFilter.h>
#include <itkCannyEdgeDetectionImageFilter.h>
#include <itkCastImageFilter.h>
#include <itkCommand.h>
#include <itkCompositeTransform.h>
#include <itkImageFileReader.h>
#include <itkImageFileWriter.h>
#include <itkLandmarkBasedTransformInitializer.h>
#include <itkLinearInterpolateImageFunction.h>
#include <itkLBFGSOptimizer.h>
#include <itkMeanSquaresPointSetToImageMetric.h>
#include <itkMesh.h>
#include <itkMeshFileReader.h>
#include <itkMeshFileWriter.h>
#include <itkPointSetToImageRegistrationMethod.h>
#include <itkPointsLocator.h>
#include <itkRigid3DTransform.h>
#include <itkSignedDanielssonDistanceMapImageFilter.h>
#include <itkTransformMeshFilter.h>

/**
 * This code serves as an example, how shape model fitting can be done using Statismo.
 * The method uses the landmark pairs to estimate the pose (rigid transform) of the image.
 * The fixed landmarks are points of the reference that was used to build the shape model
 * and the moving landmarks are the corresponding points on the target image.
 * To actually fit the new model to the image, we use the itk PointSetToImageRegistration method.
 * The fitting is not performed on the original image, but a feature image, which is obtained by performing a threshold
 * segmentation, followed by a distance transform.
 *
 */

namespace
{

//
// Internal types
//

constexpr unsigned gk_dimensions = 3;
using PointSetType = itk::PointSet<float, gk_dimensions>;
using MeshType = itk::Mesh<float, gk_dimensions>;
using PointType = itk::Point<double, 3>;
using CTImageType = itk::Image<float, gk_dimensions>;
using DistanceImageType = itk::Image<float, gk_dimensions>;
using CTImageReaderType = itk::ImageFileReader<CTImageType>;
using DistanceImageWriterType = itk::ImageFileWriter<DistanceImageType>;
using RepresenterType = itk::StandardMeshRepresenter<float, gk_dimensions>;
using MeshReaderType = itk::MeshFileReader<MeshType>;
using GradientImageType = itk::Image<itk::CovariantVector<float, gk_dimensions>, gk_dimensions>;
using MetricType = itk::MeanSquaresPointSetToImageMetric<PointSetType, DistanceImageType>;
using StatisticalModelTransformType = itk::StatisticalShapeModelTransform<MeshType, double, gk_dimensions>;
using StatisticalModelType = itk::StatisticalModel<MeshType>;
using RegistrationFilterType = itk::PointSetToImageRegistrationMethod<PointSetType, DistanceImageType>;
using OptimizerType = itk::LBFGSOptimizer;
using InterpolatorType = itk::LinearInterpolateImageFunction<DistanceImageType, double>;
using BinaryThresholdImageFilterType = itk::BinaryThresholdImageFilter<CTImageType, CTImageType>;
using DistanceMapImageFilterType = itk::SignedDanielssonDistanceMapImageFilter<CTImageType, DistanceImageType>;
using RigidTransformType = itk::VersorRigid3DTransform<double>;
using LandmarkTransformInitializerType =
  itk::LandmarkBasedTransformInitializer<RigidTransformType, DistanceImageType, DistanceImageType>;
using CompositeTransformType = itk::CompositeTransform<double, 3>;
using TransformMeshFilterType = itk::TransformMeshFilter<MeshType, MeshType, CompositeTransformType>;
using PosteriorModelBuilderType = itk::PosteriorModelBuilder<MeshType>;
using PointsLocatorType = itk::PointsLocator<MeshType::PointsContainer>;

//
// Static variables
//

constexpr int16_t gk_maxNumberOfIterations = 10000; // the maximum number of iterations to use in the optimization
constexpr double  gk_translationScale = 1;          // dynamic range of translations
constexpr double  gk_rotationScale = 0.1;           // dynamic range of rotations
constexpr double  gk_smScale = 3;                   // dynamic range of statistical model parameters

//
// This class is used to track the progress of the optimization
// (its method Execute is called in each iteration of the optimization)
//
class IterationStatusObserver : public itk::Command
{
public:
  using Self = IterationStatusObserver;
  using Superclass = itk::Command;
  using Pointer = itk::SmartPointer<Self>;

  itkNewMacro(Self);

  using OptimizerType = itk::LBFGSOptimizer;
  // using = itk::GradientDescentOptimizer OptimizerType;
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

    std::cout << "Iteration: " << ++m_iterIdx;
    std::cout << "; Value: " << optimizer->GetCachedValue();
    std::cout << "; Current Parameters: " << optimizer->GetCachedCurrentPosition() << std::endl;
  }

private:
  int m_iterIdx{ 0 };
};

//
// Utility routine
//

/**
 * read landmarks from the given file in slicer fcsv formant and return them as a list.
 *
 * The format is: label,x,y,z
 *
 * \param filename the filename
 * \return A list of itk points
 */
std::vector<PointType>
ReadLandmarks(const std::string & filename)
{
  std::vector<PointType> ptList;

  std::fstream file(filename.c_str());
  if (!file)
  {
    throw std::runtime_error("could not read landmark file");
  }

  std::string line;
  while (std::getline(file, line))
  {
    if (line.length() > 0 && line[0] == '#')
    {
      continue;
    }

    std::istringstream strstr(line);
    std::string        token;
    std::getline(strstr, token, ','); // ignore the label
    std::getline(strstr, token, ','); // get the x coord
    double pt0 = std::stod(token);
    std::getline(strstr, token, ','); // get the y coord
    double pt1 = std::stod(token);
    std::getline(strstr, token, ','); // get the z coord
    double    pt2 = std::stod(token);
    PointType pt;
    pt[0] = pt0;
    pt[1] = pt1;
    pt[2] = pt2;
    ptList.push_back(pt);
  }
  return ptList;
}

//
// Returns a new model, that is restricted to go through the proints specified in targetLandmarks..
//
StatisticalModelType::Pointer
ComputePosteriorModel(const RigidTransformType *     rigidTransform,
                      const StatisticalModelType *   statisticalModel,
                      const std::vector<PointType> & modelLandmarks,
                      const std::vector<PointType> & targetLandmarks,
                      double                         variance)
{

  // invert the transformand back transform the landmarks
  auto rinv = RigidTransformType::New();
  rigidTransform->GetInverse(rinv);

  StatisticalModelType::PointValueListType constraints;

  // We need to make sure the the points in fixed landmarks are real vertex points of the model reference.
  auto reference = statisticalModel->GetRepresenter()->GetReference();
  auto ptLocator = PointsLocatorType::New();
  ptLocator->SetPoints(reference->GetPoints());
  ptLocator->Initialize();

  assert(modelLandmarks.size() == targetLandmarks.size());
  for (unsigned i = 0; i < targetLandmarks.size(); i++)
  {
    int       closestPointId = ptLocator->FindClosestPoint(modelLandmarks[i]);
    PointType refPoint = (*reference->GetPoints())[closestPointId];

    // compensate for the rigid transformation that was applied to the model
    PointType                                targetLmAtModelPos = rinv->TransformPoint(targetLandmarks[i]);
    StatisticalModelType::PointValuePairType pointValue(refPoint, targetLmAtModelPos);
    constraints.push_back(pointValue);
  }

  auto posteriorModelBuilder = PosteriorModelBuilderType::New();
  auto posteriorModel = posteriorModelBuilder->BuildNewModelFromModel(statisticalModel, constraints, variance, false);

  return posteriorModel;
}
} // namespace

/**
 * The input to the program is:
 * -  A shape model (a statismo model that was built using the itk::MeshRepresenter
 * -  A 3D CT image
 * -  Landmarks on the reference used to build the shape model
 * -  Landmarks on the target image
 * -  A threshold that yields a rough segmentation of the structure to be fitted
 * - the name of the output
 *
 */
int
main(int argc, char * argv[])
{

  if (argc < 8)
  {
    std::cout << "usage " << argv[0]
              << " modelname ctimage fixedLandmarks movingLandmarks threshold lmVariance outputmesh" << std::endl;
    exit(-1);
  }

  const char * modelName = argv[1];
  const char * targetName = argv[2];
  const char * fixedLandmarksName = argv[3];
  const char * movingLandmarksName = argv[4];
  int16_t      threshold = std::stoi(argv[5]);
  double       lmVariance = std::stod(argv[6]);
  const char * outputMeshName = argv[7];

  // load the image to which we will fit
  auto targetReader = CTImageReaderType::New();
  targetReader->SetFileName(targetName);
  targetReader->Update();
  CTImageType::Pointer ctImage = targetReader->GetOutput();

  // We compute a binary threshold of input image to get a rough segmentation of the bony structure.
  // Then we compute a distance transform of the segmentation, which we then use for the fitting
  auto thresholdFilter = BinaryThresholdImageFilterType::New();
  thresholdFilter->SetInput(ctImage);
  thresholdFilter->SetLowerThreshold(threshold);
  auto dm = DistanceMapImageFilterType::New();
  dm->SetInput(thresholdFilter->GetOutput());
  dm->Update();
  DistanceImageType::Pointer distanceImage = dm->GetOutput();

  // read the landmarks
  auto fixedLandmarks = ReadLandmarks(fixedLandmarksName);
  auto movingLandmarks = ReadLandmarks(movingLandmarksName);

  // initialize the rigid transform
  auto rigidTransform = RigidTransformType::New();
  auto initializer = LandmarkTransformInitializerType::New();
  initializer->SetFixedLandmarks(fixedLandmarks);
  initializer->SetMovingLandmarks(movingLandmarks);
  initializer->SetTransform(rigidTransform);
  initializer->InitializeTransform();

  // load the model create a shape model transform with it
  auto representer = RepresenterType::New();
  auto model = itk::StatismoIO<MeshType>::LoadStatisticalModel(representer, modelName);

  auto constraintModel = ComputePosteriorModel(rigidTransform, model, fixedLandmarks, movingLandmarks, lmVariance);

  auto statModelTransform = StatisticalModelTransformType::New();
  statModelTransform->SetStatisticalModel(constraintModel);
  statModelTransform->SetIdentity();

  // compose the two transformation
  auto transform = CompositeTransformType::New();
  transform->AddTransform(rigidTransform);
  transform->AddTransform(statModelTransform);
  transform->SetOnlyMostRecentTransformToOptimizeOn(); //  only optimize the shape parameters, not the rigid transform
                                                       //  parameters
  // transform->SetAllTransformsToOptimizeOn(); // optimize shape and pose parameters

  // Setting up the fitting
  auto optimizer = OptimizerType::New();
  optimizer->SetMaximumNumberOfFunctionEvaluations(gk_maxNumberOfIterations);
  optimizer->MinimizeOn();

  unsigned numStatmodelParameters = statModelTransform->GetNumberOfParameters();
  unsigned totalNumParameters = rigidTransform->GetNumberOfParameters() + numStatmodelParameters;

  // set the scales of the optimizer, to compensate for potentially different scales of translation, rotation and shape
  // parameters
  OptimizerType::ScalesType scales(totalNumParameters);
  for (unsigned i = 0; i < numStatmodelParameters; i++)
  {
    scales[i] = 1.0 / (gk_smScale);
  }
  for (unsigned i = numStatmodelParameters; i < numStatmodelParameters + 3; i++)
  {
    scales[i] = 1.0 / (gk_rotationScale);
  }
  for (unsigned i = numStatmodelParameters + 3; i < statModelTransform->GetNumberOfParameters() + 6; i++)
  {
    scales[i] = 1.0 / (gk_translationScale);
  }
  optimizer->SetScales(scales);


  // set up the observer to keep track of the progress
  using ObserverType = IterationStatusObserver;
  auto observer = ObserverType::New();
  optimizer->AddObserver(itk::IterationEvent(), observer);

  // set up the metric and interpolators
  auto metric = MetricType::New();
  auto interpolator = InterpolatorType::New();

  // connect all the components
  auto registration = RegistrationFilterType::New();
  registration->SetInitialTransformParameters(transform->GetParameters());
  registration->SetInterpolator(interpolator);
  registration->SetMetric(metric);
  registration->SetOptimizer(optimizer);
  registration->SetTransform(transform);


  // we create the fixedPointSet of the registration from the reference mesh of our model.
  // As we are fitting to the 0 level set of a distance image, we set the value of the pointdata to 0.
  auto fixedPointSet = PointSetType::New();
  fixedPointSet->SetPoints(model->GetRepresenter()->GetReference()->GetPoints());
  PointSetType::PointDataContainer::Pointer points = PointSetType::PointDataContainer::New();
  points->Reserve(model->GetRepresenter()->GetReference()->GetNumberOfPoints());
  for (auto it = points->Begin(); it != points->End(); ++it)
  {
    it->Value() = 0;
  }
  fixedPointSet->SetPointData(points);

  registration->SetFixedPointSet(fixedPointSet);
  registration->SetMovingImage(distanceImage);

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

  auto transformMeshFilter = TransformMeshFilterType::New();
  transformMeshFilter->SetInput(model->GetRepresenter()->GetReference());
  transformMeshFilter->SetTransform(transform);

  // Write out the fitting result
  auto writer = itk::MeshFileWriter<MeshType>::New();
  writer->SetFileName(outputMeshName);
  writer->SetInput(transformMeshFilter->GetOutput());
  writer->Update();

  return 0;
}
