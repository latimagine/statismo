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

#include "statismo/core/HDF5Utils.h"
#include "statismo/core/Utils.h"
#include "statismo/VTK/vtkStandardMeshRepresenter.h"
#include "statismo/VTK/vtkHelper.h"

#include <vtkCellArray.h>
#include <vtkCellData.h>
#include <vtkDataSetAttributes.h>
#include <vtkPoints.h>
#include <vtkPointData.h>
#include <vtkPolyDataReader.h>
#include <vtkNew.h>

#include <memory>

namespace statismo
{

vtkStandardMeshRepresenter::vtkStandardMeshRepresenter(DatasetConstPointerType reference)
  : vtkStandardMeshRepresenter()
{
  this->SetReference(reference);
}

vtkStandardMeshRepresenter *
vtkStandardMeshRepresenter::CloneImpl() const
{
  return Create(m_reference);
}

void
vtkStandardMeshRepresenter::Load(const H5::Group & fg)
{
  vtkSmartPointer<vtkPolyData> ref;

  auto repName = HDF5Utils::ReadStringAttribute(fg, "name");
  if (repName == "vtkPolyDataRepresenter" || repName == "itkMeshRepresenter")
  {
    ref = LoadRefLegacy(fg);
  }
  else
  {
    ref = LoadRef(fg);
  }

  this->SetReference(ref);
}

vtkStandardMeshRepresenter::DatasetPointerType
vtkStandardMeshRepresenter::LoadRef(const H5::Group & fg) const
{
  statismo::MatrixType vertexMat;
  HDF5Utils::ReadMatrix(fg, "./points", vertexMat);

  using UIntMatrixType = statismo::GenericEigenTraits<unsigned int>::MatrixType;
  UIntMatrixType cellsMat;
  HDF5Utils::ReadMatrixOfType<unsigned int>(fg, "./cells", cellsMat);

  // create the reference from this information
  auto ref = DatasetPointerType::New();

  unsigned nVertices = vertexMat.cols();
  unsigned nCells = cellsMat.cols();

  auto pcoords = vtkSmartPointer<vtkFloatArray>::New();
  pcoords->SetNumberOfComponents(3);
  pcoords->SetNumberOfTuples(nVertices);
  for (unsigned i = 0; i < nVertices; i++)
  {
    pcoords->SetTuple3(i, vertexMat(0, i), vertexMat(1, i), vertexMat(2, i));
  }
  auto points = vtkSmartPointer<vtkPoints>::New();
  points->SetData(pcoords);

  ref->SetPoints(points);

  auto     cell = vtkSmartPointer<vtkCellArray>::New();
  unsigned cellDim = cellsMat.rows();
  for (unsigned i = 0; i < nCells; i++)
  {
    cell->InsertNextCell(cellDim);
    for (unsigned d = 0; d < cellDim; d++)
    {
      cell->InsertCellPoint(cellsMat(d, i));
    }
  }
  if (cellDim == 2)
  {
    ref->SetLines(cell);
  }
  else
  {
    ref->SetPolys(cell);
  }

  // read the point and cell data
  assert(ref->GetPointData());
  if (HDF5Utils::ExistsObjectWithName(fg, "pointData"))
  {
    H5::Group pdGroup = fg.openGroup("./pointData");
    if (HDF5Utils::ExistsObjectWithName(pdGroup, "scalars"))
    {
      ref->GetPointData()->SetScalars(GetAsDataArray(pdGroup, "scalars"));
    }
    if (HDF5Utils::ExistsObjectWithName(pdGroup, "vectors"))
    {
      ref->GetPointData()->SetVectors(GetAsDataArray(pdGroup, "vectors"));
    }
    if (HDF5Utils::ExistsObjectWithName(pdGroup, "normals"))
    {
      ref->GetPointData()->SetNormals(GetAsDataArray(pdGroup, "normals"));
    }
  }

  if (HDF5Utils::ExistsObjectWithName(fg, "cellData"))
  {
    H5::Group cdGroup = fg.openGroup("./cellData");
    assert(ref->GetCellData());

    if (HDF5Utils::ExistsObjectWithName(cdGroup, "scalars"))
    {
      ref->GetPointData()->SetScalars(GetAsDataArray(cdGroup, "scalars"));
    }
    if (HDF5Utils::ExistsObjectWithName(cdGroup, "vectors"))
    {
      ref->GetPointData()->SetVectors(GetAsDataArray(cdGroup, "vectors"));
    }
    if (HDF5Utils::ExistsObjectWithName(cdGroup, "normals"))
    {
      ref->GetPointData()->SetNormals(GetAsDataArray(cdGroup, "normals"));
    }
  }
  return ref;
}


vtkStandardMeshRepresenter::DatasetPointerType
vtkStandardMeshRepresenter::LoadRefLegacy(const H5::Group & fg) const
{
  std::string tmpfilename = statismo::utils::CreateTmpName(".vtk");

  HDF5Utils::GetFileFromHDF5(fg, "./reference", tmpfilename.c_str());

  vtkNew<vtkPolyDataReader> reader;
  reader->SetFileName(tmpfilename.c_str());
  reader->Update();
  statismo::utils::RemoveFile(tmpfilename);
  if (reader->GetErrorCode() != 0)
  {
    throw StatisticalModelException((std::string("Could not read file ") + tmpfilename).c_str(), Status::IO_ERROR);
  }
  return reader->GetOutput();
}

void
vtkStandardMeshRepresenter::Save(const H5::Group & fg) const
{
  using namespace H5;

  statismo::MatrixType vertexMat = statismo::MatrixType::Zero(3, m_reference->GetNumberOfPoints());
  for (unsigned i = 0; i < m_reference->GetNumberOfPoints(); i++)
  {
    PointType pt = m_reference->GetPoint(i);
    for (unsigned d = 0; d < 3; d++)
    {
      vertexMat(d, i) = pt[d];
    }
  }
  HDF5Utils::WriteMatrix(fg, "./points", vertexMat);

  // check the dimensionality of a face (i.e. the number of points it has). We assume that
  // all the cells are the same.
  unsigned numPointsPerCell{ 0 };
  if (m_reference->GetNumberOfCells() > 0)
  {
    numPointsPerCell = m_reference->GetCell(0)->GetNumberOfPoints();
  }

  using UIntMatrixType = statismo::GenericEigenTraits<unsigned int>::MatrixType;
  UIntMatrixType facesMat = UIntMatrixType::Zero(numPointsPerCell, m_reference->GetNumberOfCells());
  for (unsigned i = 0; i < m_reference->GetNumberOfCells(); i++)
  {
    vtkCell * cell = m_reference->GetCell(i);
    assert(numPointsPerCell == cell->GetNumberOfPoints());
    for (unsigned d = 0; d < numPointsPerCell; d++)
    {
      facesMat(d, i) = cell->GetPointIds()->GetId(d);
    }
  }

  HDF5Utils::WriteMatrixOfType<unsigned int>(fg, "./cells", facesMat);
  auto pdGroup = fg.createGroup("pointData");

  vtkPointData * pd = m_reference->GetPointData();
  if (pd && pd->GetScalars())
  {
    vtkDataArray * scalars = pd->GetScalars();
    WriteDataArray(pdGroup, "scalars", scalars);
  }

  if (pd && pd->GetVectors())
  {
    vtkDataArray * vectors = pd->GetVectors();
    WriteDataArray(pdGroup, "vectors", vectors);
  }

  if (pd && pd->GetNormals())
  {
    vtkDataArray * normals = pd->GetNormals();
    WriteDataArray(pdGroup, "normals", normals);
  }

  auto          cdGroup = fg.createGroup("cellData");
  vtkCellData * cd = m_reference->GetCellData();
  if (cd && cd->GetScalars())
  {
    vtkDataArray * scalars = cd->GetScalars();
    WriteDataArray(cdGroup, "scalars", scalars);
  }

  if (cd && cd->GetVectors())
  {
    vtkDataArray * vectors = cd->GetVectors();
    WriteDataArray(cdGroup, "vectors", vectors);
  }

  if (cd && cd->GetNormals())
  {
    vtkDataArray * normals = cd->GetNormals();
    WriteDataArray(cdGroup, "normals", normals);
  }
}

statismo::VectorType
vtkStandardMeshRepresenter::PointToVector(const PointType & pt) const
{
  return Eigen::Map<const VectorTypeDoublePrecision>(pt.Data(), 3).cast<float>();
}

statismo::VectorType
vtkStandardMeshRepresenter::SampleToSampleVector(DatasetConstPointerType sample) const
{
  assert(m_reference);

  auto mutableSample = const_cast<vtkPolyData *>(sample);

  VectorType sampleVec = VectorType::Zero(m_reference->GetNumberOfPoints() * 3);
  for (unsigned i = 0; i < m_reference->GetNumberOfPoints(); i++)
  {
    for (unsigned j = 0; j < 3; j++)
    {
      sampleVec(MapPointIdToInternalIdx(i, j)) = mutableSample->GetPoint(i)[j];
    }
  }
  return sampleVec;
}

vtkStandardMeshRepresenter::DatasetPointerType
vtkStandardMeshRepresenter::SampleVectorToSample(const VectorType & sample) const
{
  assert(m_reference);

  auto reference = const_cast<vtkPolyData *>(m_reference.GetPointer());
  auto pd = vtkSmartPointer<vtkPolyData>::New();
  pd->DeepCopy(reference);

  vtkPoints * points = pd->GetPoints();
  for (unsigned i = 0; i < reference->GetNumberOfPoints(); i++)
  {
    vtkPoint pt;
    for (unsigned d = 0; d < GetDimensions(); d++)
    {
      pt[d] = sample(MapPointIdToInternalIdx(i, d));
    }
    points->SetPoint(i, pt.Data());
  }
  return pd;
}

vtkStandardMeshRepresenter::ValueType
vtkStandardMeshRepresenter::PointSampleFromSample(DatasetConstPointerType sample, unsigned ptid) const
{
  auto mutableSample = const_cast<vtkPolyData *>(sample);
  if (ptid >= mutableSample->GetNumberOfPoints())
  {
    throw StatisticalModelException("invalid ptid provided to PointSampleFromSample");
  }
  return vtkPoint(mutableSample->GetPoints()->GetPoint(ptid));
}

statismo::VectorType
vtkStandardMeshRepresenter::PointSampleToPointSampleVector(const ValueType & v) const
{
  VectorType vec(GetDimensions());
  for (unsigned i = 0; i < GetDimensions(); i++)
  {
    vec(i) = v[i];
  }
  return vec;
}

vtkStandardMeshRepresenter::ValueType
vtkStandardMeshRepresenter::PointSampleVectorToPointSample(const VectorType & v) const
{
  ValueType value;
  for (unsigned i = 0; i < GetDimensions(); i++)
  {
    value[i] = v(i);
  }
  return value;
}

unsigned
vtkStandardMeshRepresenter::GetPointIdForPoint(const PointType & pt) const
{
  assert(m_reference);
  return this->m_reference->FindPoint(const_cast<double *>(pt.Data()));
}

unsigned
vtkStandardMeshRepresenter::GetNumberOfPoints() const
{
  assert(m_reference);
  return this->m_reference->GetNumberOfPoints();
}

vtkSmartPointer<vtkDataArray>
vtkStandardMeshRepresenter::GetAsDataArray(const H5::Group & group, const std::string & name)
{
  using DoubleMatrixType = statismo::GenericEigenTraits<double>::MatrixType;
  DoubleMatrixType m;
  HDF5Utils::ReadMatrixOfType<double>(group, name.c_str(), m);

  // we open the dataset once more to be able to read its attribute
  auto matrixDs = group.openDataSet(name.c_str());

  auto dataArray = helper::vtkDataTypeIdToArray(HDF5Utils::ReadIntAttribute(matrixDs, "datatype"));
  FillDataArray(m, dataArray);
  return dataArray;
}

void
vtkStandardMeshRepresenter::FillDataArray(const statismo::GenericEigenTraits<double>::MatrixType & m,
                                          vtkDataArray *                                           dataArray)
{
  unsigned numComponents = m.rows();
  unsigned numTuples = m.cols();
  dataArray->SetNumberOfComponents(numComponents);
  dataArray->SetNumberOfTuples(numTuples);

  for (unsigned i = 0; i < numTuples; i++)
  {
    dataArray->InsertTuple(i, m.col(i).data());
  }
}

void
vtkStandardMeshRepresenter::SetReference(DatasetConstPointerType reference)
{
  m_reference->DeepCopy(const_cast<vtkPolyData *>(reference));

  // set the domain
  DomainType::DomainPointsListType ptList;
  for (unsigned i = 0; i < m_reference->GetNumberOfPoints(); i++)
  {
    ptList.emplace_back(m_reference->GetPoint(i));
  }
  m_domain = DomainType(ptList);
}


void
vtkStandardMeshRepresenter::WriteDataArray(const H5::H5Location & group,
                                           const std::string &    name,
                                           const vtkDataArray *   ds) const
{
  auto     dataArray = const_cast<vtkDataArray *>(ds);
  unsigned numComponents = dataArray->GetNumberOfComponents();
  unsigned numTuples = dataArray->GetNumberOfTuples();
  using DoubleMatrixType = statismo::GenericEigenTraits<double>::MatrixType;
  DoubleMatrixType m = DoubleMatrixType::Zero(numComponents, numTuples);

  for (unsigned i = 0; i < numTuples; i++)
  {
    double * tuple = dataArray->GetTuple(i);
    for (unsigned d = 0; d < numComponents; d++)
    {
      m(d, i) = tuple[d];
    }
  }

  HDF5Utils::WriteIntAttribute(HDF5Utils::WriteMatrixOfType<double>(group, name.c_str(), m),
                               "datatype",
                               statismo::helper::vtkDataTypeIdToStatismoDataTypeId(dataArray->GetDataType()));
}

} // namespace statismo
