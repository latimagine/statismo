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

#ifndef __STATISMO_VTK_HELPER_H_
#define __STATISMO_VTK_HELPER_H_

#include "statismo/core/CommonTypes.h"
#include "statismo/core/Exceptions.h"

#include <vtkType.h>
#include <vtkCharArray.h>
#include <vtkDoubleArray.h>
#include <vtkFloatArray.h>
#include <vtkLongArray.h>
#include <vtkIntArray.h>
#include <vtkShortArray.h>
#include <vtkUnsignedCharArray.h>
#include <vtkUnsignedIntArray.h>
#include <vtkUnsignedLongArray.h>
#include <vtkUnsignedShortArray.h>

#include <sstream>

namespace statismo::helper
{
inline int
vtkDataTypeIdToStatismoDataTypeId(int vtkDataTypeId)
{
  switch (vtkDataTypeId)
  {
    case VTK_UNSIGNED_CHAR:
      return statismo::gk_unsignedChar;
    case VTK_SIGNED_CHAR:
      return statismo::gk_signedChar;
    case VTK_FLOAT:
      return statismo::gk_float;
    case VTK_DOUBLE:
      return statismo::gk_double;
    case VTK_UNSIGNED_INT:
      return statismo::gk_unsignedInt;
    case VTK_INT:
      return statismo::gk_signedInt;
    case VTK_UNSIGNED_SHORT:
      return statismo::gk_unsignedShort;
    case VTK_SHORT:
      return statismo::gk_signedShort;
    case VTK_UNSIGNED_LONG:
      return statismo::gk_unsignedLong;
    case VTK_LONG:
      return statismo::gk_signedLong;
    default:
      break;
  }
  throw statismo::StatisticalModelException("Unsupported data type", Status::INVALID_DATA_ERROR);
}

inline int
vtkStatismoDataTypeIdToVtkDataTypeId(unsigned statismoDataTypeId)
{
  switch (statismoDataTypeId)
  {
    case statismo::gk_unsignedChar:
      return VTK_UNSIGNED_CHAR;
    case statismo::gk_signedChar:
      return VTK_SIGNED_CHAR;
    case statismo::gk_float:
      return VTK_FLOAT;
    case statismo::gk_double:
      return VTK_DOUBLE;
    case statismo::gk_unsignedInt:
      return VTK_UNSIGNED_INT;
    case statismo::gk_signedInt:
      return VTK_INT;
    case statismo::gk_unsignedShort:
      return VTK_UNSIGNED_SHORT;
    case statismo::gk_signedShort:
      return VTK_SHORT;
    case statismo::gk_unsignedLong:
      return VTK_UNSIGNED_LONG;
    case statismo::gk_signedLong:
      return VTK_LONG;
    default:
      break;
  }
  throw statismo::StatisticalModelException("Unsupported data type", Status::INVALID_DATA_ERROR);
}

inline vtkSmartPointer<vtkDataArray>
vtkDataTypeIdToArray(int vtkDataTypeId)
{
  switch (vtkDataTypeId)
  {
    case statismo::gk_unsignedChar:
      return vtkSmartPointer<vtkUnsignedCharArray>::New();
    case statismo::gk_signedChar:
      return vtkSmartPointer<vtkCharArray>::New();
    case statismo::gk_float:
      return vtkSmartPointer<vtkFloatArray>::New();
    case statismo::gk_double:
      return vtkSmartPointer<vtkDoubleArray>::New();
    case statismo::gk_unsignedInt:
      return vtkSmartPointer<vtkUnsignedIntArray>::New();
    case statismo::gk_signedInt:
      return vtkSmartPointer<vtkIntArray>::New();
    case statismo::gk_unsignedShort:
      return vtkSmartPointer<vtkUnsignedShortArray>::New();
    case statismo::gk_signedShort:
      return vtkSmartPointer<vtkShortArray>::New();
    case statismo::gk_signedLong:
      return vtkSmartPointer<vtkLongArray>::New();
    case statismo::gk_unsignedLong:
      return vtkSmartPointer<vtkUnsignedLongArray>::New();
    default:
      break;
  }

  throw StatisticalModelException("Unsupported data type for dataArray", Status::INVALID_DATA_ERROR);
}
} // namespace statismo::helper

#endif
