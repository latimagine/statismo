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

#include "statismo/VTK/vtkUnstructuredGridRepresenter.h"
#include "statismo/core/HDF5Utils.h"
#include "statismo/core/Utils.h"
#include "statismo/VTK/vtkHelper.h"

#include <vtkDataArray.h>
#include <vtkPoints.h>
#include <vtkPointData.h>
#include <vtkXMLUnstructuredGridReader.h>
#include <vtkXMLUnstructuredGridWriter.h>

namespace statismo
{

vtkUnstructuredGridRepresenter::vtkUnstructuredGridRepresenter()
  : m_reference{ DatasetPointerType::New() }
  , m_pdTransform{ vtkSmartPointer<vtkTransformPolyDataFilter>::New() }
  , m_alignment{ AlignmentType::NONE }
{}

vtkUnstructuredGridRepresenter::vtkUnstructuredGridRepresenter(DatasetConstPointerType reference,
                                                               AlignmentType           alignment)
  : vtkUnstructuredGridRepresenter()
{
  m_alignment = alignment;

  this->SetReference(reference);
}

vtkUnstructuredGridRepresenter *
vtkUnstructuredGridRepresenter::CloneImpl() const
{
  // this works since Create deep copies the reference
  return Create(m_reference, m_alignment);
}

void
vtkUnstructuredGridRepresenter::Load(const H5::Group & fg)
{
  auto tmpfilename = statismo::utils::CreateTmpName(".vtk");
  auto uw = statismo::MakeStackUnwinder([=]() { statismo::utils::RemoveFile(tmpfilename); });

  statismo::hdf5utils::GetFileFromHDF5(fg, "./reference", tmpfilename.c_str());

  m_alignment = static_cast<AlignmentType>(statismo::hdf5utils::ReadInt(fg, "./alignment"));
  this->SetReference(ReadDataset(tmpfilename));
}

void
vtkUnstructuredGridRepresenter::Save(const H5::Group & fg) const
{
  std::string tmpfilename = statismo::utils::CreateTmpName(".vtk");
  auto        uw = statismo::MakeStackUnwinder([=]() { statismo::utils::RemoveFile(tmpfilename); });

  WriteDataset(tmpfilename, this->m_reference);

  statismo::hdf5utils::DumpFileToHDF5(tmpfilename.c_str(), fg, "./reference");
  statismo::hdf5utils::WriteInt(fg, "./alignment", static_cast<int>(m_alignment));
}

statismo::VectorType
vtkUnstructuredGridRepresenter::PointToVector(const PointType & pt) const
{
  return Eigen::Map<const VectorTypeDoublePrecision>(pt.data(), 3).cast<float>();
}

statismo::VectorType
vtkUnstructuredGridRepresenter::SampleToSampleVector(DatasetConstPointerType sample) const
{
  assert(m_reference);

  auto mutableSample = const_cast<vtkUnstructuredGrid *>(sample);
  auto deformationVectors = mutableSample->GetPointData()->GetVectors();

  statismo::VectorType sampleVec = statismo::VectorType::Zero(m_reference->GetNumberOfPoints() * 3);
  for (unsigned i = 0; i < m_reference->GetNumberOfPoints(); i++)
  {
    for (unsigned j = 0; j < 3; j++)
    {
      sampleVec(MapPointIdToInternalIdx(i, j)) = deformationVectors->GetTuple(i)[j];
    }
  }
  return sampleVec;
}

vtkUnstructuredGridRepresenter::DatasetPointerType
vtkUnstructuredGridRepresenter::SampleVectorToSample(const statismo::VectorType & sampleVec) const
{
  assert(m_reference);

  auto reference = const_cast<vtkUnstructuredGrid *>(m_reference.GetPointer());

  auto pd = vtkSmartPointer<vtkUnstructuredGrid>::New();
  pd->DeepCopy(reference);

  auto * deformationVectors = pd->GetPointData()->GetVectors();

  for (unsigned i = 0; i < reference->GetNumberOfPoints(); i++)
  {
    statismo::vtkPoint pt;
    for (unsigned d = 0; d < GetDimensions(); d++)
    {
      pt[d] = sampleVec(MapPointIdToInternalIdx(i, d));
    }
    deformationVectors->SetTuple(i, pt.data());
  }

  return pd;
}

vtkUnstructuredGridRepresenter::ValueType
vtkUnstructuredGridRepresenter::PointSampleFromSample(DatasetConstPointerType sample, unsigned ptid) const
{
  auto mutableSample = const_cast<vtkUnstructuredGrid *>(sample);
  if (ptid >= mutableSample->GetNumberOfPoints())
  {
    throw statismo::StatisticalModelException("invalid ptid provided to PointSampleFromSample");
  }
  return statismo::vtkPoint(mutableSample->GetPointData()->GetVectors()->GetTuple(ptid));
}

statismo::VectorType
vtkUnstructuredGridRepresenter::PointSampleToPointSampleVector(const ValueType & v) const
{
  statismo::VectorType vec(GetDimensions());
  for (unsigned i = 0; i < GetDimensions(); i++)
  {
    vec(i) = v[i];
  }
  return vec;
}

vtkUnstructuredGridRepresenter::ValueType
vtkUnstructuredGridRepresenter::PointSampleVectorToPointSample(const statismo::VectorType & v) const
{
  ValueType value;
  for (unsigned i = 0; i < GetDimensions(); i++)
  {
    value[i] = v(i);
  }
  return value;
}

unsigned
vtkUnstructuredGridRepresenter::GetPointIdForPoint(const PointType & pt) const
{
  assert(m_reference);
  return this->m_reference->FindPoint(const_cast<double *>(pt.data()));
}

unsigned
vtkUnstructuredGridRepresenter::GetNumberOfPoints() const
{
  assert(m_reference);
  return this->m_reference->GetNumberOfPoints();
}


vtkUnstructuredGridRepresenter::DatasetPointerType
vtkUnstructuredGridRepresenter::ReadDataset(const std::string & filename)
{
  vtkNew<vtkXMLUnstructuredGridReader> reader;
  reader->SetFileName(filename.c_str());
  reader->Update();

  if (reader->GetErrorCode() != 0)
  {
    throw statismo::StatisticalModelException((std::string("Could not read file ") + filename).c_str(),
                                              Status::IO_ERROR);
  }
  return reader->GetOutput();
}

void
vtkUnstructuredGridRepresenter::WriteDataset(const std::string & filename, DatasetConstPointerType pd)
{
  vtkNew<vtkXMLUnstructuredGridWriter> writer;
  writer->SetFileName(filename.c_str());
  writer->SetInputData(const_cast<vtkUnstructuredGrid *>(pd));
  writer->Update();

  if (writer->GetErrorCode() != 0)
  {
    throw statismo::StatisticalModelException((std::string("Could not read file ") + filename).c_str(),
                                              Status::IO_ERROR);
  }
}

void
vtkUnstructuredGridRepresenter::SetReference(DatasetConstPointerType reference)
{
  m_reference->DeepCopy(const_cast<vtkUnstructuredGrid *>(reference));

  // set the domain
  vtkDataArray *                   deformationVectors = m_reference->GetPointData()->GetVectors();
  DomainType::DomainPointsListType ptList;
  for (unsigned i = 0; i < m_reference->GetNumberOfPoints(); i++)
  {
    ptList.emplace_back(deformationVectors->GetTuple(i));
  }
  m_domain = DomainType(ptList);
}

} // namespace statismo
