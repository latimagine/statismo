/*
 * This file is part of the statismo library.
 *
 * Copyright (c) 2019 Laboratory of Medical Information Processing
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

#ifndef __STATIMO_ITK_UTILS_H_
#define __STATIMO_ITK_UTILS_H_

#include <itkObject.h>
#include <itkDirectory.h>
#include <itkIdentityTransform.h>
#include <itkTransformMeshFilter.h>

#include <string>
#include <sstream>
#include <vector>

namespace statismo::itk
{
struct ExceptionHandler
{
  explicit ExceptionHandler(const ::itk::Object & o)
    : m_obj{ o }
  {}

  void
  operator()(const std::string & str) const
  {
    std::ostringstream message;
    message << "itk::ERROR: " << m_obj.GetNameOfClass() << "(" << &m_obj << "): " << str;
    throw ::itk::ExceptionObject{ __FILE__, __LINE__, message.str().c_str(), ITK_LOCATION };
  }

private:
  const ::itk::Object & m_obj;
};

inline std::vector<std::string>
GetDirFiles(const std::string & dir, const std::string & extension = ".*")
{
  auto directory = ::itk::Directory::New();
  directory->Load(dir.c_str());

  std::vector<std::string> files;
  for (unsigned i = 0; i < directory->GetNumberOfFiles(); i++)
  {
    const char * filename = directory->GetFile(i);
    if (extension == ".*" || std::string(filename).find(extension) != std::string::npos)
    {
      files.emplace_back(filename);
    }
  }

  return files;
}

template <typename DataType>
typename DataType::Pointer
CloneMesh(typename DataType::Pointer pMesh)
{
  using IdentityTransformType = ::itk::IdentityTransform<float, DataType::PointDimension>;
  using TransformFilterType = ::itk::TransformMeshFilter<DataType, DataType, IdentityTransformType>;
  auto transformMeshFilter = TransformFilterType::New();
  auto identityTransform = IdentityTransformType::New();
  transformMeshFilter->SetInput(pMesh);
  transformMeshFilter->SetTransform(identityTransform);
  transformMeshFilter->Update();

  return transformMeshFilter->GetOutput();
}

} // namespace statismo::itk

#endif