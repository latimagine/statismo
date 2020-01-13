#include <statismo/ITK/itkPosteriorModelBuilder.h>

template <class DataType>
std::vector<typename DataType::PointType>
readLandmarksFile(std::string path)
{
  std::vector<typename DataType::PointType> vLandmarks;

  std::ifstream file;
  try
  {
    file.exceptions(std::ifstream::failbit | std::ifstream::badbit);
    file.open(path.c_str(), std::ifstream::in);

    std::string line;
    while (getline(file, line))
    {
      if (line != "")
      {
        // reading files with windows EOL on linux results in the \r not being removed from the line ending
        if (*line.rbegin() == '\r')
        {
          line.erase(line.length() - 1, 1);
        }
        // typedef boost::tokenizer<boost::escaped_list_separator<char> > TokenizerType;
        // TokenizerType t(line);
        // TODO: Replace with a real csv tokenizer that can handle coma in escaped string
        auto                                   t = statismo::utils::Split<','>(line);
        typename DataType::PointType           p;
        typename DataType::PointType::Iterator pointIter = p.Begin();
        // The first element is the description/name and will be ignored
        for (auto i = ++t.begin(); i != t.end(); ++i, ++pointIter)
        {
          try
          {
            float fCoordValue = statismo::utils::LexicalCast<float>(*i);
            if (pointIter == p.End())
            {
              // ignore the last point if it is equal to 0 in the 2D case (and it really is the last point)
              if (p.Size() == 2 && ++i == t.end())
              {
                if (fCoordValue == 0)
                {
                  break;
                }
                else
                {
                  itkGenericExceptionMacro(<< "The last point in the 2D case has to be 0 in the following line: '"
                                           << line << "' (file: " << path << ")");
                }
              }
              else
              {
                itkGenericExceptionMacro(<< "Too many point components were found in this line: '" << line
                                         << "' (file: " << path << ")");
              }
            }
            *pointIter = fCoordValue;
          }
          catch (const std::bad_cast & e)
          {
            itkGenericExceptionMacro(<< "Could not parse '" << (*i) << "' to a float in this line: '" << line
                                     << "' in the file '" << path << "'");
          }
        }
        if (pointIter != p.End())
        {
          itkGenericExceptionMacro(<< "Not enough point components were found in this line: '" << line
                                   << "' (file: " << path << ")");
        }
        vLandmarks.push_back(p);
      }
    }
  }
  catch (std::ifstream::failure e)
  {
    if (file.eof() == false)
    {
      throw std::ifstream::failure("Failed to read a file: '" + path + "'");
    }
  }

  return vLandmarks;
}



template <class DataType, class StatisticalModelType>
typename StatisticalModelType::Pointer
buildPosteriorDeformationModel(typename StatisticalModelType::Pointer pModel,
                               const std::string &                    strFixedLandmarksFileName,
                               const std::string &                    strMovingLandmarksFileName,
                               const double                           dLandmarksVariance)
{
  typedef std::vector<typename DataType::PointType> PointVector;
  PointVector vFixedLandmarks = readLandmarksFile<DataType>(strFixedLandmarksFileName);
  PointVector vMovingLandmarks = readLandmarksFile<DataType>(strMovingLandmarksFileName);

  if (vFixedLandmarks.size() != vMovingLandmarks.size())
  {
    itkGenericExceptionMacro(<< "There have to be an equal number of fixed and moving Landmarks.")
  }

  typename StatisticalModelType::PointValueListType vConstraints;

  typename PointVector::iterator pFixed = vFixedLandmarks.begin();
  for (typename PointVector::iterator pMoving = vMovingLandmarks.begin(); pMoving != vMovingLandmarks.end();
       ++pMoving, ++pFixed)
  {
    typename DataType::PixelType pxDisplacement;
    for (unsigned i = 0; i < pFixed->Size(); ++i)
    {
      pxDisplacement[i] = (*pMoving)[i] - (*pFixed)[i];
    }
    typename StatisticalModelType::PointValuePairType pointValue(*pFixed, pxDisplacement);
    vConstraints.push_back(pointValue);
  }

  typedef itk::PosteriorModelBuilder<DataType> PosteriorModelBuilderType;
  typename PosteriorModelBuilderType::Pointer  pPosteriorModelBuilder = PosteriorModelBuilderType::New();
  return pPosteriorModelBuilder->BuildNewModelFromModel(pModel, vConstraints, dLandmarksVariance, false);
}


template <class DataType, class StatisticalModelType>
typename StatisticalModelType::Pointer
buildPosteriorShapeModel(typename StatisticalModelType::Pointer pModel,
                         typename DataType::Pointer             pMesh,
                         const double                           dVariance)
{
  if (pMesh->GetNumberOfPoints() != pModel->GetRepresenter()->GetReference()->GetNumberOfPoints())
  {
    itkGenericExceptionMacro(<< "The provided Mesh is not in correspondence.")
  }

  typename StatisticalModelType::PointValueListType vConstraints;

  typename DataType::PointsContainer::Iterator pFixed = pModel->GetRepresenter()->GetReference()->GetPoints()->Begin();
  for (typename DataType::PointsContainer::Iterator pMoving = pMesh->GetPoints()->Begin();
       pMoving != pMesh->GetPoints()->End();
       ++pMoving, ++pFixed)
  {
    typename StatisticalModelType::PointValuePairType pointValue(pFixed->Value(), pMoving->Value());
    vConstraints.push_back(pointValue);
  }

  typedef itk::PosteriorModelBuilder<DataType> PosteriorModelBuilderType;
  typename PosteriorModelBuilderType::Pointer  pPosteriorModelBuilder = PosteriorModelBuilderType::New();
  return pPosteriorModelBuilder->BuildNewModelFromModel(pModel, vConstraints, dVariance, false);
}

template <class DataType, class StatisticalModelType, class PointsLocatorType>
typename StatisticalModelType::Pointer
buildPosteriorShapeModel(typename StatisticalModelType::Pointer pModel,
                         const std::string &                    strFixedLandmarksFileName,
                         const std::string &                    strMovingLandmarksFileName,
                         const double                           dLandmarksVariance)
{
  std::vector<typename DataType::PointType> vFixedLandmarks = readLandmarksFile<DataType>(strFixedLandmarksFileName);
  std::vector<typename DataType::PointType> vMovingLandmarks = readLandmarksFile<DataType>(strMovingLandmarksFileName);

  if (vFixedLandmarks.size() != vMovingLandmarks.size())
  {
    itkGenericExceptionMacro(<< "There have to be an equal number of fixed and moving Landmarks.")
  }

  typename DataType::Pointer          pReference = pModel->GetRepresenter()->GetReference();
  typename PointsLocatorType::Pointer pPointLocator = PointsLocatorType::New();
  pPointLocator->SetPoints(pReference->GetPoints());
  pPointLocator->Initialize();

  const typename DataType::PointsContainer *        pcReferenceMeshPoints = pReference->GetPoints();
  typename StatisticalModelType::PointValueListType vConstraints;

  for (unsigned i = 0; i < vFixedLandmarks.size(); ++i)
  {

    unsigned                     iClosestPointId = pPointLocator->FindClosestPoint(vFixedLandmarks[i]);
    typename DataType::PointType refPoint = pcReferenceMeshPoints->at(iClosestPointId);

    // compensate for the rigid transformation that was applied to the model
    typename StatisticalModelType::PointValuePairType pointValue(refPoint, vMovingLandmarks[i]);
    vConstraints.push_back(pointValue);
  }

  typedef itk::PosteriorModelBuilder<DataType> PosteriorModelBuilderType;
  typename PosteriorModelBuilderType::Pointer  pPosteriorModelBuilder = PosteriorModelBuilderType::New();
  return pPosteriorModelBuilder->BuildNewModelFromModel(pModel, vConstraints, dLandmarksVariance, false);
}