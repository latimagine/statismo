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

#ifndef __STATISMO_VTK_NDPIXEL_H_
#define __STATISMO_VTK_NDPIXEL_H_

#include "statismo/core/Exceptions.h"

#include <sstream>
#include <memory>

namespace statismo
{

/**
 * \brief Helper class that represents a vtkPixel of arbitrary type and dimension
 * In vtk a pixel is just of type  T*. The statismo library relies on a proper
 * copy semantics, and hence requires such a wrapper.
 */
class vtkNDPixel
{
public:
  explicit vtkNDPixel(unsigned dimensions)
    : m_pixel{std::make_unique<double []>(dimensions)}
    , m_dimensions(dimensions)
  {}

  vtkNDPixel(const double * x, unsigned dimensions)
    : m_pixel{std::make_unique<double []>(dimensions)}
    , m_dimensions(dimensions)
  {
    for (unsigned d = 0; d < dimensions; d++) {
      m_pixel[d] = x[d];
    }
  }

  ~vtkNDPixel() = default;

  vtkNDPixel(const vtkNDPixel& rhs) : vtkNDPixel{rhs.m_pixel.get(), rhs.m_dimensions}
  {
  }

  vtkNDPixel(vtkNDPixel&& rhs) noexcept : m_pixel{std::move(rhs.m_pixel)}, m_dimensions{rhs.m_dimensions}
  {
  }

  vtkNDPixel& operator=(vtkNDPixel rhs)
  {
      swap(rhs);
      return *this;
  }

  void swap(vtkNDPixel& rhs) noexcept
  {
      using std::swap;
      swap(m_pixel, rhs.m_pixel);
      swap(m_dimensions, rhs.m_dimensions);
  }

  double & operator[](unsigned i)
  {
    return const_cast<double&>(static_cast<const vtkNDPixel&>(*this)[i]);
  }

  const double & operator[](unsigned i) const
  {
    if (i >= m_dimensions)
    {
      std::ostringstream os;
      os << "Invalid index for vtkPixel (index = " << i << ")";
      throw statismo::StatisticalModelException(os.str().c_str(), statismo::Status::OUT_OF_RANGE_ERROR);
    }
    else
    {
      return m_pixel[i];
    }
  }

private:
  std::unique_ptr<double[]> m_pixel;
  unsigned m_dimensions;
};

} // namespace statismo

namespace std
{
  template <>
  void swap<statismo::vtkNDPixel>(statismo::vtkNDPixel& lhs, statismo::vtkNDPixel& rhs) noexcept
  {
      lhs.swap(rhs);
  }
}

#endif
