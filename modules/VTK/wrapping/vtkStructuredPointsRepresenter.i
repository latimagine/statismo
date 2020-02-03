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

#include "Core.i"
 
%{
#include "statismo/VTK/vtkStandardImageRepresenter.h"
#include <vtkStructuredPoints.h>
%}

%template(vtkSmartPointer_vtkSP) vtkSmartPointer<vtkStructuredPoints>;

%template(Representer_vtkSP) statismo::Representer<vtkStructuredPoints>;

%template(vtkStandardImageRepresenter_F2) statismo::vtkStandardImageRepresenter<float, 2>;
%template(GenericFactory_vtkSIR_F2) statismo::GenericFactory<statismo::vtkStandardImageRepresenter<float, 2>>;
%template(RepresenterBase_vtkSP_vtkSIR_F2) statismo::RepresenterBase<vtkStructuredPoints, statismo::vtkStandardImageRepresenter<float, 2>>;


%template(vtkStandardImageRepresenter_F3) statismo::vtkStandardImageRepresenter<float, 3>;
%template(GenericFactory_vtkSIR_F3) statismo::GenericFactory<statismo::vtkStandardImageRepresenter<float, 3>>;
%template(RepresenterBase_vtkSP_vtkSIR_F3) statismo::RepresenterBase<vtkStructuredPoints, statismo::vtkStandardImageRepresenter<float, 3>>;

%template(vtkStandardImageRepresenter_D2) statismo::vtkStandardImageRepresenter<double, 2>;
%template(GenericFactory_vtkSIR_D2) statismo::GenericFactory<statismo::vtkStandardImageRepresenter<double, 2>>;
%template(RepresenterBase_vtkSP_vtkSIR_D2) statismo::RepresenterBase<vtkStructuredPoints, statismo::vtkStandardImageRepresenter<double, 2>>;

%template(vtkStandardImageRepresenter_D3) statismo::vtkStandardImageRepresenter<double, 3>;
%template(GenericFactory_vtkSIR_D3) statismo::GenericFactory<statismo::vtkStandardImageRepresenter<double, 3>>;
%template(RepresenterBase_vtkSP_vtkSIR_D3) statismo::RepresenterBase<vtkStructuredPoints, statismo::vtkStandardImageRepresenter<double, 3>>;

namespace statismo {

class vtkNDPixel
{
	public:
	vtkNDPixel(unsigned dimensions);
	vtkNDPixel(const double * x, unsigned dimensions);
};

%rename(RepresenterTraits_vtkSP) RepresenterTraits<vtkStructuredPoints>;  
struct RepresenterTraits<vtkStructuredPoints>
{
  typedef vtkSmartPointer<vtkStructuredPoints> DatasetPointerType;
  typedef const vtkStructuredPoints * DatasetConstPointerType;

  typedef vtkPoint PointType;
  typedef vtkNDPixel ValueType;
};

template <class TPixel, unsigned TDimensions>
class vtkStandardImageRepresenter : public RepresenterBase<vtkStructuredPoints, vtkStandardImageRepresenter<TScalar, PIXEL_DIMENSIONS>> {
	public:
		%newobject Create; 
		static vtkStandardImageRepresenter* Create(); 
		static vtkStandardImageRepresenter* Create(const vtkStructuredPoints* reference);

		const DomainType& GetDomain() const;
		unsigned GetNumberOfPoints() const;
		unsigned GetPointIdForPoint(const vtkPoint & pt) const;
	private:
		vtkStandardImageRepresenter();
		vtkStandardImageRepresenter(const vtkStructuredPoints* reference);
};
}