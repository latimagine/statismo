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
%{
#include "statismo/core/DataItem.h"
#include "statismo/core/DataManager.h"
#include "statismo/core/DataManagerWithSurrogates.h"
#include "statismo/core/StatisticalModel.h"
#include "statismo/core/PosteriorModelBuilder.h"
#include "statismo/core/ReducedVarianceModelBuilder.h"
#include "statismo/core/PCAModelBuilder.h"
#include "statismo/core/CommonTypes.h"
#include "statismo/core/IO.h"
#include "statismo/core/Representer.h"
#include "statismo/VTK/vtkPoint.h"
#include "statismo/VTK/vtkNDPixel.h"

#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkUnstructuredGrid.h>
#include <vtkStructuredPoints.h>

#include <list>
#include <string>
#include <type_traits>
#include <memory>
%}

%include "std_shared_ptr.i"
%include "typemaps.i"
%include "std_string.i"
%include "std_list.i"
%include "std_vector.i"
%include "std_pair.i"
%include "std_vector.i"
%include "carrays.i"

%include "statismoTypemaps.i"

//
// Here you will find all stuff that needs to be instantiated
// for each representer
//
// \warning Do not use the following features as it
//          is not or only partially supported by
//          swig-3.0:
//          - alias (using directive)
//          - std::unique_ptr
//

//////////////////////////////////////////////////////
// Point used by all representers
//////////////////////////////////////////////////////

namespace statismo { 

class vtkPoint {
    public:
        vtkPoint(double x, double y, double z);
};

class vtkNDPixel
{
    public:
        vtkNDPixel(unsigned dimensions);
        vtkNDPixel(const double * x, unsigned dimensions);
};

}

%template(PointPointValuePair) std::pair<statismo::vtkPoint, statismo::vtkPoint>;
%template(PointPointValueList) std::list<std::pair<statismo::vtkPoint, statismo::vtkPoint> >;
%template(PointIdPointValuePair) std::pair<unsigned, statismo::vtkPoint>;
%template(PointIdPointValueList) std::list<std::pair<unsigned, statismo::vtkPoint> >;

%template(PointPixelValuePair) std::pair<statismo::vtkPoint, statismo::vtkNDPixel>;
%template(PointPixelValueList) std::list<std::pair<statismo::vtkPoint, statismo::vtkNDPixel> >;
%template(PointIdPixelValuePair) std::pair<unsigned, statismo::vtkNDPixel>;
%template(PointIdPixelValueList) std::list<std::pair<unsigned, statismo::vtkNDPixel> >;

//////////////////////////////////////////////////////
// Exception handling
//////////////////////////////////////////////////////

namespace statismo {

class StatisticalModelException {
    public:
        StatisticalModelException(const char* message);
};
}

%exception {
   try {
      $action
   } catch (const statismo::StatisticalModelException &e) {
      PyErr_SetString(PyExc_RuntimeError, const_cast<char*>(e.what()));
      return NULL;
   }
}

//////////////////////////////////////////////////////
// ModelInfo
//////////////////////////////////////////////////////

%template(StrPair) std::pair<std::string, std::string>;
%template(StrPairList) std::list<std::pair<std::string, std::string> >;

namespace statismo {
class BuilderInfo {
    public:
        typedef std::pair<std::string, std::string> KeyValuePair;
        typedef std::list<KeyValuePair> KeyValueList;

        const KeyValueList& GetDataInfo() const;
        const KeyValueList& GetParameterInfo() const;
};
}

%template(BuilderInfoList) std::vector<statismo::BuilderInfo>;

namespace statismo {
class ModelInfo {
    public:
        typedef std::vector<BuilderInfo> BuilderInfoList;
        
        const statismo::MatrixType& GetScoresMatrix() const;
        virtual void Save(const H5::H5Location& publicFg) const;
        virtual void Load(const H5::H5Location& publicFg);
        BuilderInfoList GetBuilderInfoList() const;
};
}

//////////////////////////////////////////////////////
// Domain
//////////////////////////////////////////////////////

%template(PointList) std::vector<statismo::vtkPoint>;
%template(IdList) std::vector<unsigned int>;

namespace statismo {
template <typename PointType>
class Domain {
    public:
        typedef std::vector<PointType> DomainPointsListType;
        
        DomainPointsListType GetDomainPoints() const;
        unsigned GetNumberOfPoints() const;
};
}

%template(PointDomain) statismo::Domain<statismo::vtkPoint>;
%template(IdDomain) statismo::Domain<unsigned int>;

////////////////////////////////////////////////////
// Representer
/////////////////////////////////////////////////////

namespace statismo {

template <typename T>
struct RepresenterTraits;

%rename(RepresenterTraits_vtkPD) RepresenterTraits<vtkPolyData>; 
struct RepresenterTraits<vtkPolyData> {
    typedef const vtkPolyData* DatasetConstPointerType;
    typedef vtkSmartPointer<vtkPolyData> DatasetPointerType;
    typedef vtkPoint PointType;
    typedef vtkPoint ValueType;    
};

%rename(RepresenterTraits_vtkUG) RepresenterTraits<vtkUnstructuredGrid>; 
struct RepresenterTraits<vtkUnstructuredGrid> {
    typedef const vtkUnstructuredGrid* DatasetConstPointerType;
    typedef vtkSmartPointer<vtkUnstructuredGrid> DatasetPointerType;
    typedef vtkPoint PointType;
    typedef vtkPoint ValueType;    
};

%rename(RepresenterTraits_vtkSP) RepresenterTraits<vtkStructuredPoints>; 
struct RepresenterTraits<vtkStructuredPoints> {
    typedef const vtkStructuredPoints* DatasetConstPointerType;
    typedef vtkSmartPointer<vtkStructuredPoints> DatasetPointerType;
    typedef vtkPoint PointType;
    typedef vtkNDPixel ValueType;    
};

template<class T>
class Representer  {
    public:
        typedef typename RepresenterTraits<T>::DatasetPointerType DatasetPointerType;
        typedef typename RepresenterTraits<T>::DatasetConstPointerType DatasetConstPointerType;
        typedef typename RepresenterTraits<T>::PointType PointType;
        typedef typename RepresenterTraits<T>::ValueType ValueType;

        virtual unsigned
        GetPointIdForPoint(const PointType & point) const = 0;

        virtual const statismo::Domain<PointType> &
        GetDomain() const = 0;    

        virtual void Save(const H5::Group & fg) const = 0;

        virtual DatasetConstPointerType GetReference() const = 0;

        virtual unsigned GetDimensions() const = 0;
};

}

%template(Representer_vtkPD) statismo::Representer<vtkPolyData>;
%template(Representer_vtkUG) statismo::Representer<vtkUnstructuredGrid>;
%template(Representer_vtkSP) statismo::Representer<vtkStructuredPoints>;

namespace statismo {

template <typename T, typename Derived>
class RepresenterBase : public statismo::Representer<T> /*, public statismo::GenericFactory<Derived>*/
{
    public:
        virtual unsigned GetDimensions() const;
};

}

//////////////////////////////////////////////////////
// DataManager
//////////////////////////////////////////////////////

%shared_ptr(statismo::DataItem<vtkPolyData>)
%shared_ptr(statismo::DataItemBase<vtkPolyData,statismo::BasicDataItem<vtkPolyData>>)
%shared_ptr(statismo::DataItemBase<vtkPolyData,statismo::DataItemWithSurrogates<vtkPolyData>>)
%shared_ptr(statismo::BasicDataItem<vtkPolyData>)
%shared_ptr(statismo::DataItemWithSurrogates<vtkPolyData>)

%shared_ptr(statismo::DataItem<vtkUnstructuredGrid>)
%shared_ptr(statismo::DataItemBase<vtkUnstructuredGrid,statismo::BasicDataItem<vtkUnstructuredGrid>>)
%shared_ptr(statismo::DataItemBase<vtkUnstructuredGrid,statismo::DataItemWithSurrogates<vtkUnstructuredGrid>>)
%shared_ptr(statismo::BasicDataItem<vtkUnstructuredGrid>)
%shared_ptr(statismo::DataItemWithSurrogates<vtkUnstructuredGrid>)

%shared_ptr(statismo::DataItem<vtkStructuredPoints>)
%shared_ptr(statismo::DataItemBase<vtkStructuredPoints,statismo::BasicDataItem<vtkStructuredPoints>>)
%shared_ptr(statismo::DataItemBase<vtkStructuredPoints,statismo::DataItemWithSurrogates<vtkStructuredPoints>>)
%shared_ptr(statismo::BasicDataItem<vtkStructuredPoints>)
%shared_ptr(statismo::DataItemWithSurrogates<vtkStructuredPoints>)

namespace statismo {

template <typename T>
class DataItem {
    public:
        typedef Representer<T> RepresenterType;
        typedef typename RepresenterType::DatasetPointerType DatasetPointerType;

        virtual std::string
        GetDatasetURI() const = 0;

        virtual statismo::VectorType
        GetSampleVector() const = 0;

        virtual DatasetPointerType
        GetSample() const = 0;
};

}

%inline %{
    typedef std::shared_ptr<statismo::DataItem<vtkPolyData>> DataItemVtkPDPtr;
    typedef std::shared_ptr<statismo::DataItem<vtkUnstructuredGrid>> DataItemVtkUGPtr;
    typedef std::shared_ptr<statismo::DataItem<vtkStructuredPoints>> DataItemVtkSPPtr;
%}

%template(DataItem_vtkPD) statismo::DataItem<vtkPolyData>;
%template(DataItemList_vtkPD) std::list<DataItemVtkPDPtr>;

%template(DataItem_vtkUG) statismo::DataItem<vtkUnstructuredGrid>;
%template(DataItemList_vtkUG) std::list<DataItemVtkUGPtr>;

%template(DataItem_vtkSP) statismo::DataItem<vtkStructuredPoints>;
%template(DataItemList_vtkSP) std::list<DataItemVtkSPPtr>;

namespace statismo {

template <typename T, typename Derived>
class DataItemBase
  : public DataItem<T>
  /*, public GenericFactory<Derived>*/ {
    public:
        typedef typename DataItem<T>::RepresenterType RepresenterType;
        typedef typename DataItem<T>::DatasetPointerType DatasetPointerType;

    
    protected:
        DataItemBase(const RepresenterType * representer, std::string uri, statismo::VectorType sampleVector);
        DataItemBase(const RepresenterType * representer);
};

}

%template(DataItemBase_vtkPD_BasicDataItem_vtkPD) statismo::DataItemBase<vtkPolyData, statismo::BasicDataItem<vtkPolyData>>;
%template(DataItemBase_vtkPD_DataItemWithSurrogates_vtkPD) statismo::DataItemBase<vtkPolyData, statismo::DataItemWithSurrogates<vtkPolyData>>;

%template(DataItemBase_vtkUG_BasicDataItem_vtkUG) statismo::DataItemBase<vtkUnstructuredGrid, statismo::BasicDataItem<vtkUnstructuredGrid>>;
%template(DataItemBase_vtkUG_DataItemWithSurrogates_vtkUG) statismo::DataItemBase<vtkUnstructuredGrid, statismo::DataItemWithSurrogates<vtkUnstructuredGrid>>;

%template(DataItemBase_vtkSP_BasicDataItem_vtkSP) statismo::DataItemBase<vtkStructuredPoints, statismo::BasicDataItem<vtkStructuredPoints>>;
%template(DataItemBase_vtkSP_DataItemWithSurrogates_vtkSP) statismo::DataItemBase<vtkStructuredPoints, statismo::DataItemWithSurrogates<vtkStructuredPoints>>;

namespace statismo {

template <typename T>
class BasicDataItem : public DataItemBase<T, BasicDataItem<T>> {
    public:
        typedef Representer<T> RepresenterType;
        typedef typename RepresenterType::DatasetPointerType DatasetPointerType;

        %newobject Create; 
        static BasicDataItem* Create(const RepresenterType* representer, const std::string& filename, const statismo::VectorType& sampleVector); 
        static BasicDataItem* Create(const RepresenterType* representer);

        DatasetPointerType GetSample() const;

        // Uncomment if you got problem with vtk smart pointer to extend
        // the interface with raw pointer
        // %extend {
        //     DatasetRawPointerType GetSampleDataset() const {
        //         DatasetRawPointerType* ds = DatasetRawPointerType::New();
        //         ds->DeepCopy(GetSample());
        //         return ds;
        //     }
        // }

        std::string GetDatasetURI() const;
        statismo::VectorType GetSampleVector() const;

    private:
        BasicDataItem(const RepresenterType* representer, const std::string& filename, const statismo::VectorType& sampleVector);
        BasicDataItem(const RepresenterType* representer);
};

}

%template(BasicDataItem_vtkPD) statismo::BasicDataItem<vtkPolyData>;
%traits_swigtype(statismo::BasicDataItem<vtkPolyData>); // workaround for a swig bug with const ptr
%fragment(SWIG_Traits_frag(statismo::BasicDataItem<vtkPolyData>));  // workaround for a swig bug with const ptr
%template(BasicDataItemList_vtkPD) std::list<const statismo::BasicDataItem<vtkPolyData> *>;

%template(BasicDataItem_vtkUG) statismo::BasicDataItem<vtkUnstructuredGrid>;
%traits_swigtype(statismo::BasicDataItem<vtkUnstructuredGrid>); // workaround for a swig bug with const ptr
%fragment(SWIG_Traits_frag(statismo::BasicDataItem<vtkUnstructuredGrid>));  // workaround for a swig bug with const ptr
%template(BasicDataItemList_vtkUG) std::list<const statismo::BasicDataItem<vtkUnstructuredGrid> *>;

%template(BasicDataItem_vtkSP) statismo::BasicDataItem<vtkStructuredPoints>;
%traits_swigtype(statismo::BasicDataItem<vtkStructuredPoints>); // workaround for a swig bug with const ptr
%fragment(SWIG_Traits_frag(statismo::BasicDataItem<vtkStructuredPoints>));  // workaround for a swig bug with const ptr
%template(BasicDataItemList_vtkSP) std::list<const statismo::BasicDataItem<vtkStructuredPoints> *>;

namespace statismo {

template <typename T>
class DataItemWithSurrogates : public DataItemBase<T, DataItemWithSurrogates<T>> {
    public:
        typedef Representer<T> RepresenterType;
        typedef typename RepresenterType::DatasetPointerType DatasetPointerType;

        %newobject Create; 
        static DataItemWithSurrogates* Create(const RepresenterType * representer,
                            std::string             datasetURI,
                            statismo::VectorType              sampleVector,
                            std::string             surrogateFilename,
                            statismo::VectorType              surrogateVector); 
        static DataItemWithSurrogates* Create(const RepresenterType* representer);

        DatasetPointerType GetSample() const;

        // Uncomment if you got problem with vtk smart pointer to extend
        // the interface with raw pointer
        // %extend {
        //     DatasetRawPointerType GetSampleDataset() {
        //         DatasetRawPointerType* ds = DatasetRawPointerType::New();
        //         ds->DeepCopy(GetSample());
        //         return ds;
        //     }
        // }

        std::string GetDatasetURI() const;
        statismo::VectorType GetSampleVector() const;
        statismo::VectorType GetSurrogateVector() const;
        std::string GetSurrogateFilename() const;

    private:
        DataItemWithSurrogates(const RepresenterType * representer,
                            std::string             datasetURI,
                            statismo::VectorType              sampleVector,
                            std::string             surrogateFilename,
                            statismo::VectorType              surrogateVector);
        DataItemWithSurrogates(const RepresenterType* representer);
};

}

%template(DataItemWithSurrogates_vtkPD) statismo::DataItemWithSurrogates<vtkPolyData>;
%traits_swigtype(statismo::DataItemWithSurrogates<vtkPolyData>); // workaround for a swig bug with const ptr
%fragment(SWIG_Traits_frag(statismo::DataItemWithSurrogates<vtkPolyData>));  // workaround for a swig bug with const ptr
%template(DataItemWithSurrogatesList_vtkPD) std::list<const statismo::DataItemWithSurrogates<vtkPolyData> *>;

%template(DataItemWithSurrogates_vtkUG) statismo::DataItemWithSurrogates<vtkUnstructuredGrid>;
%traits_swigtype(statismo::DataItemWithSurrogates<vtkUnstructuredGrid>); // workaround for a swig bug with const ptr
%fragment(SWIG_Traits_frag(statismo::DataItemWithSurrogates<vtkUnstructuredGrid>));  // workaround for a swig bug with const ptr
%template(DataItemWithSurrogatesList_vtkUG) std::list<const statismo::DataItemWithSurrogates<vtkUnstructuredGrid> *>;

%template(DataItemWithSurrogates_vtkSP) statismo::DataItemWithSurrogates<vtkStructuredPoints>;
%traits_swigtype(statismo::DataItemWithSurrogates<vtkStructuredPoints>); // workaround for a swig bug with const ptr
%fragment(SWIG_Traits_frag(statismo::DataItemWithSurrogates<vtkStructuredPoints>));  // workaround for a swig bug with const ptr
%template(DataItemWithSurrogatesList_vtkSP) std::list<const statismo::DataItemWithSurrogates<vtkStructuredPoints> *>;

namespace statismo {
template <typename T>
class CrossValidationFold {
    public:
        typedef DataItem<T> DataItemType;
        typedef std::list<std::shared_ptr<DataItemType>> DataItemListType;

        CrossValidationFold();
        DataItemListType GetTrainingData() const;
        DataItemListType GetTestingData() const;
};
}

%template(CrossValidationFold_vtkPD) statismo::CrossValidationFold<vtkPolyData>;
%template(CrossValidationFoldList_vtkPD) std::list<statismo::CrossValidationFold<vtkPolyData>>;

%template(CrossValidationFold_vtkUG) statismo::CrossValidationFold<vtkUnstructuredGrid>;
%template(CrossValidationFoldList_vtkUG) std::list<statismo::CrossValidationFold<vtkUnstructuredGrid>>;

%template(CrossValidationFold_vtkSP) statismo::CrossValidationFold<vtkStructuredPoints>;
%template(CrossValidationFoldList_vtkSP) std::list<statismo::CrossValidationFold<vtkStructuredPoints>>;

namespace statismo {

template <typename T>
class DataManager
{
    public:
        typedef Representer<T> RepresenterType;
        typedef typename RepresenterType::DatasetConstPointerType DatasetConstPointerType;
        typedef CrossValidationFold<T> CrossValidationFoldType;
        typedef std::list<CrossValidationFoldType> CrossValidationFoldListType;
        typedef DataItem<T> DataItemType;
        typedef std::list<std::shared_ptr<DataItemType>> DataItemListType;

        virtual CrossValidationFoldListType
        GetCrossValidationFolds(unsigned nFolds, bool isRandomized) const = 0;

        virtual unsigned
        GetNumberOfSamples() const = 0;

        virtual void
        AddDataset(DatasetConstPointerType dataset, const std::string & uri) = 0;

        virtual void
        Save(const std::string & filename) const = 0;

        virtual DataItemListType
        GetData() const = 0;
};

}

%template(DataManager_vtkPD) statismo::DataManager<vtkPolyData>;
%template(DataManager_vtkUG) statismo::DataManager<vtkUnstructuredGrid>;
%template(DataManager_vtkSP) statismo::DataManager<vtkStructuredPoints>;

namespace statismo {

template <typename T, typename Derived>
class DataManagerBase
  : public DataManager<T>
  /*, public GenericFactory<Derived>*/
{
    public:

    protected:
        explicit DataManagerBase(const RepresenterType * representer);
};

}

%template(DataManagerBase_vtkPD_BasicDataManager_vtkPD) statismo::DataManagerBase<vtkPolyData, statismo::BasicDataManager<vtkPolyData>>;
%template(DataManagerBase_vtkPD_DataManagerWithSurrogates_vtkPD) statismo::DataManagerBase<vtkPolyData, statismo::DataManagerWithSurrogates<vtkPolyData>>;

%template(DataManagerBase_vtkUG_BasicDataManager_vtkUG) statismo::DataManagerBase<vtkUnstructuredGrid, statismo::BasicDataManager<vtkUnstructuredGrid>>;
%template(DataManagerBase_vtkUG_DataManagerWithSurrogates_vtkUG) statismo::DataManagerBase<vtkUnstructuredGrid, statismo::DataManagerWithSurrogates<vtkUnstructuredGrid>>;

%template(DataManagerBase_vtkSP_BasicDataManager_vtkSP) statismo::DataManagerBase<vtkStructuredPoints, statismo::BasicDataManager<vtkStructuredPoints>>;
%template(DataManagerBase_vtkSP_DataManagerWithSurrogates_vtkSP) statismo::DataManagerBase<vtkStructuredPoints, statismo::DataManagerWithSurrogates<vtkStructuredPoints>>;

namespace statismo {

template <typename T>
class BasicDataManager : public DataManagerBase<T, BasicDataManager<T>>
{
    public:
        typedef Representer<T> RepresenterType;
        typedef typename RepresenterType::DatasetConstPointerType DatasetConstPointerType;
        typedef CrossValidationFold<T> CrossValidationFoldType;
        typedef std::list<CrossValidationFoldType> CrossValidationFoldListType;
        typedef DataItem<T> DataItemType;
        typedef std::list<std::shared_ptr<DataItemType>> DataItemListType;

        %newobject Create;
        static BasicDataManager* Create(const RepresenterType*);

        // Overcome unique pointer problem
        %ignore BasicDataManager::Load(RepresenterType* representer, const char* filename);
        %extend {
            %newobject Load;
            static DataManagerBase<T, BasicDataManager<T>>* Load(RepresenterType* representer, const char* filename) {
                return statismo::BasicDataManager<T>::Load(representer, filename).release();
            }
        }
    
        void AddDataset(DatasetConstPointerType dataset, const std::string& URI);
        unsigned GetNumberOfSamples() const;
        void Save(const std::string & filename);

        CrossValidationFoldListType GetCrossValidationFolds(unsigned nFolds, bool randomize = true) const;
        DataItemListType GetData() const;

    private:
        BasicDataManager(const RepresenterType * representer);
};

}

%template(BasicDataManager_vtkPD) statismo::BasicDataManager<vtkPolyData>;
%template(BasicDataManager_vtkUG) statismo::BasicDataManager<vtkUnstructuredGrid>;
%template(BasicDataManager_vtkSP) statismo::BasicDataManager<vtkStructuredPoints>;

namespace statismo {

template <typename T>
class DataManagerWithSurrogates : public DataManagerBase<T, DataManagerWithSurrogates<T>>
{
    public:
        typedef Representer<T> RepresenterType;
        typedef typename RepresenterType::DatasetConstPointerType DatasetConstPointerType;
        typedef CrossValidationFold<T> CrossValidationFoldType;
        typedef std::list<CrossValidationFoldType> CrossValidationFoldListType;
        typedef DataItem<T> DataItemType;
        typedef std::list<std::shared_ptr<DataItemType>> DataItemListType;

        %newobject Create;
        static DataManagerWithSurrogates* Create(const RepresenterType*, const std::string& surrogTypeFilename);

        // Overcome unique pointer problem
        %ignore DataManagerWithSurrogates::Load(RepresenterType* representer, const char* filename);
        %extend {
            %newobject Load;
            static DataManagerBase<T, DataManagerWithSurrogates<T>>* Load(RepresenterType* representer, const char* h5Filename, const char* surrogateFilename) {
                return statismo::DataManagerWithSurrogates<T>::Load(representer, h5Filename, surrogateFilename).release();
            }
        }
    
        void AddDataset(DatasetConstPointerType dataset, const std::string& URI);
        void AddDatasetWithSurrogates(DatasetConstPointerType ds,
                           const std::string &     datasetURI,
                           const std::string &     surrogateFilename);
        unsigned GetNumberOfSamples() const;
        void Save(const std::string & filename);

        /** cross validation functionality */
        CrossValidationFoldListType GetCrossValidationFolds(unsigned nFolds, bool randomize = true) const;
        DataItemListType GetData() const;
    private:
        DataManagerWithSurrogates(const RepresenterType * r, const std::string & filename);
};

}

%template(DataManagerWithSurrogates_vtkPD) statismo::DataManagerWithSurrogates<vtkPolyData>;
%template(DataManagerWithSurrogates_vtkUG) statismo::DataManagerWithSurrogates<vtkUnstructuredGrid>;
%template(DataManagerWithSurrogates_vtkSP) statismo::DataManagerWithSurrogates<vtkStructuredPoints>;

//////////////////////////////////////////////////////
// StatisticalModel
//////////////////////////////////////////////////////

namespace statismo {
template <class T>
class StatisticalModel /*: public GenericFactory<StatisticalModel<T>>*/
{
    public:
        typedef Representer<T> RepresenterType;
        typedef typename RepresenterType::DatasetPointerType DatasetPointerType;
        typedef typename RepresenterType::DatasetConstPointerType DatasetConstPointerType;
        typedef typename RepresenterType::ValueType ValueType;
        typedef typename RepresenterType::PointType PointType;
        typedef Domain<PointType> DomainType;
        typedef unsigned PointIdType;
        typedef std::pair<PointType, ValueType> PointValuePairType;
        typedef std::pair<unsigned, ValueType> PointIdValuePairType;
        typedef std::list<PointValuePairType> PointValueListType;
        typedef std::list<PointIdValuePairType> PointIdValueListType;

        %newobject Create;
        static StatisticalModel* Create(const RepresenterType * representer,
                                        statismo::VectorType              m,
                                        const statismo::MatrixType &      orthonormalPCABasis,
                                        statismo::VectorType              pcaVariance,
                                        double                  noiseVariance);
        const RepresenterType* GetRepresenter() const;
        const DomainType& GetDomain() const;

        ValueType EvaluateSampleAtPoint(DatasetConstPointerType sample, const PointType& point) const;
        DatasetPointerType DrawMean() const;
        ValueType DrawMeanAtPoint(const PointType& pt) const;
        ValueType DrawMeanAtPoint(unsigned) const;

        DatasetPointerType DrawSample(bool addNoise = false) const;
        DatasetPointerType DrawSample(const statismo::VectorType& coeffs, bool addNoise = false) const;
        ValueType DrawSampleAtPoint(const statismo::VectorType& coeffs, const PointType& pt, bool addNoise = false) const;
        ValueType DrawSampleAtPoint(const statismo::VectorType& coeffs, unsigned ptId, bool addNoise = false) const;

        DatasetPointerType DrawPCABasisSample(unsigned componentNumber) const;

        statismo::VectorType ComputeCoefficients(DatasetConstPointerType ds) const;
        statismo::VectorType ComputeCoefficientsForSampleVector(const statismo::VectorType& sample) const;
        statismo::VectorType ComputeCoefficientsForPointValues(const PointValueListType&  pointValues, double pointValueNoiseVariance = 0.0) const;
        statismo::VectorType ComputeCoefficientsForPointIDValues(const PointIdValueListType&  pointValues, double pointValueNoiseVariance = 0.0) const;

        double ComputeLogProbability(DatasetConstPointerType ds) const;
        double ComputeProbability(DatasetConstPointerType ds) const;
        double ComputeMahalanobisDistance(DatasetConstPointerType ds) const;

        statismo::MatrixType GetCovarianceAtPoint(const PointType& pt1, const PointType& pt2) const;
        statismo::MatrixType GetCovarianceAtPoint(unsigned ptId1, unsigned ptId2) const;

        unsigned GetNumberOfPrincipalComponents() const;
        statismo::VectorType DrawSampleVector(const statismo::VectorType& coefficients, bool addNoise = false) const;
        const statismo::MatrixType& GetPCABasisMatrix() const;
        statismo::MatrixType GetOrthonormalPCABasisMatrix() const;
        const statismo::VectorType& GetPCAVarianceVector() const;
        const statismo::VectorType& GetMeanVector() const;

        const statismo::ModelInfo& GetModelInfo() const;
        void SetModelInfo(const statismo::ModelInfo& modelInfo);
        float GetNoiseVariance() const;

    private:
        StatisticalModel(const RepresenterType * representer,
                        statismo::VectorType              m,
                        const statismo::MatrixType &      orthonormalPCABasis,
                        statismo::VectorType              pcaVariance,
                        double                  noiseVariance);

};
}

%template(StatisticalModel_vtkPD) statismo::StatisticalModel<vtkPolyData>;
%template(StatisticalModel_vtkUG) statismo::StatisticalModel<vtkUnstructuredGrid>;
%template(StatisticalModel_vtkSP) statismo::StatisticalModel<vtkStructuredPoints>;

//////////////////////////////////////////////////////
// Model builders
//////////////////////////////////////////////////////

namespace statismo
{

template <typename T>
class ModelBuilder
{
    public:
        typedef Representer<T> RepresenterType;
        typedef StatisticalModel<T> StatisticalModelType;
        typedef BasicDataManager<T> DataManagerType;
        typedef typename DataManagerType::DataItemListType DataItemListType;
    protected:
        ModelBuilder() = default;
};

}

%template(ModelBuilder_vtkPD) statismo::ModelBuilder<vtkPolyData>;
%template(ModelBuilder_vtkUG) statismo::ModelBuilder<vtkUnstructuredGrid>;
%template(ModelBuilder_vtkSP) statismo::ModelBuilder<vtkStructuredPoints>;

namespace statismo {

template <typename T, typename Derived>
class ModelBuilderBase
  : public ModelBuilder<T>
  /*, public GenericFactory<Derived>*/
{
};

}

%template(ModelBuilderBase_vtkPD_PCAModelBuilder_vtkPD) statismo::ModelBuilderBase<vtkPolyData, statismo::PCAModelBuilder<vtkPolyData>>;
%template(ModelBuilderBase_vtkPD_PosteriorModelBuilder_vtkPD) statismo::ModelBuilderBase<vtkPolyData, statismo::PosteriorModelBuilder<vtkPolyData>>;
%template(ModelBuilderBase_vtkPD_ReducedVarianceModelBuilder_vtkPD) statismo::ModelBuilderBase<vtkPolyData, statismo::ReducedVarianceModelBuilder<vtkPolyData>>;

%template(ModelBuilderBase_vtkUG_PCAModelBuilder_vtkUG) statismo::ModelBuilderBase<vtkUnstructuredGrid, statismo::PCAModelBuilder<vtkUnstructuredGrid>>;
%template(ModelBuilderBase_vtkUG_PosteriorModelBuilder_vtkUG) statismo::ModelBuilderBase<vtkUnstructuredGrid, statismo::PosteriorModelBuilder<vtkUnstructuredGrid>>;
%template(ModelBuilderBase_vtkUG_ReducedVarianceModelBuilder_vtkUG) statismo::ModelBuilderBase<vtkUnstructuredGrid, statismo::ReducedVarianceModelBuilder<vtkUnstructuredGrid>>;

%template(ModelBuilderBase_vtkSP_PCAModelBuilder_vtkSP) statismo::ModelBuilderBase<vtkStructuredPoints, statismo::PCAModelBuilder<vtkStructuredPoints>>;
%template(ModelBuilderBase_vtkSP_PosteriorModelBuilder_vtkSP) statismo::ModelBuilderBase<vtkStructuredPoints, statismo::PosteriorModelBuilder<vtkStructuredPoints>>;
%template(ModelBuilderBase_vtkSP_ReducedVarianceModelBuilder_vtkSP) statismo::ModelBuilderBase<vtkStructuredPoints, statismo::ReducedVarianceModelBuilder<vtkStructuredPoints>>;

namespace statismo {

template <typename T>
class PCAModelBuilder : public ModelBuilderBase<T, PCAModelBuilder<T>> {
    public:

        typedef ModelBuilderBase<T, PCAModelBuilder<T>> Superclass;
        typedef typename Superclass::DataManagerType DataManagerType;
        typedef typename Superclass::StatisticalModelType StatisticalModelType;
        typedef typename DataManagerType::DataItemListType DataItemListType;

        enum class EigenValueMethod
        {
            JACOBI_SVD,
            SELF_ADJOINT_EIGEN_SOLVER
        };

        %newobject Create;
        static PCAModelBuilder* Create();

        // Overcome unique pointer problem
        %ignore PCAModelBuilder::BuildNewModel(const DataItemListType &, double, bool, EigenValueMethod);
        %extend {
            %newobject BuildNewModel;
            StatisticalModelType* BuildNewModel(const DataItemListType & sampleDataList,
                double                   noiseVariance,
                bool                     computeScores = true,
                EigenValueMethod         method = EigenValueMethod::JACOBI_SVD) {
                    return $self->BuildNewModel(sampleDataList, noiseVariance, computeScores, method).release();
            }
        }
    
    private:
        PCAModelBuilder();

};
}

%template(PCAModelBuilder_vtkPD) statismo::PCAModelBuilder<vtkPolyData>;
%template(PCAModelBuilder_vtkUG) statismo::PCAModelBuilder<vtkUnstructuredGrid>;
%template(PCAModelBuilder_vtkSP) statismo::PCAModelBuilder<vtkStructuredPoints>;

namespace statismo {

template <typename T>
class PosteriorModelBuilder : public ModelBuilderBase<T, PosteriorModelBuilder<T>> {
    public:
        typedef ModelBuilderBase<T, PosteriorModelBuilder<T>> Superclass;
        typedef typename Superclass::DataManagerType DataManagerType;
        typedef typename DataManagerType::DataItemListType DataItemListType;
        typedef typename Superclass::StatisticalModelType StatisticalModelType;
        typedef typename StatisticalModelType::PointValueListType PointValueListType;
        typedef typename StatisticalModelType::PointValuePairType PointValuePairType;

        %newobject Create;
        static PosteriorModelBuilder* Create();

        // Overcome unique pointer problem
        %ignore PosteriorModelBuilder::BuildNewModel(const DataItemListType &, const PointValueListType &, double, double);
        %ignore PosteriorModelBuilder::BuildNewModelFromModel(const StatisticalModelType *, const PointValueListType &, double, bool);
        %extend {
            %newobject BuildNewModel;
            StatisticalModelType* BuildNewModel(const DataItemListType &   dataItemList,
                const PointValueListType & pointValues,
                double                     pointValueNoiseVariance,
                double                     noiseVariance) {
                return $self->BuildNewModel(dataItemList, pointValues, pointValueNoiseVariance, noiseVariance).release();
            }

            %newobject BuildNewModelFromModel;
            StatisticalModelType* BuildNewModelFromModel(const StatisticalModelType * model,
                         const PointValueListType &   pointValues,
                         double                       pointValueNoiseVariance,
                         bool                         computeScores = true) {
                return $self->BuildNewModelFromModel(model, pointValues, pointValueNoiseVariance, computeScores).release();
            }
        }
    private:
        PosteriorModelBuilder();
};
}

%template(PosteriorModelBuilder_vtkPD) statismo::PosteriorModelBuilder<vtkPolyData>;
%template(PosteriorModelBuilder_vtkUG) statismo::PosteriorModelBuilder<vtkUnstructuredGrid>;
%template(PosteriorModelBuilder_vtkSP) statismo::PosteriorModelBuilder<vtkStructuredPoints>;

namespace statismo {

template <typename Representer>
class ReducedVarianceModelBuilder : public ModelBuilderBase<Representer, ReducedVarianceModelBuilder<Representer>> {
    public:
        typedef ModelBuilder<Representer> Superclass;
        typedef StatisticalModel<Representer> StatisticalModelType;

        %newobject Create;
        static ReducedVarianceModelBuilder* Create();

        // Overcome unique pointer problem
        %ignore ReducedVarianceModelBuilder::BuildNewModelWithLeadingComponents(const StatisticalModelType *, unsigned);
        %ignore ReducedVarianceModelBuilder::BuildNewModelWithVariance(const StatisticalModelType *, double);
        %extend {
            %newobject BuildNewModelWithLeadingComponents;
            StatisticalModelType* BuildNewModelWithLeadingComponents(const StatisticalModelType * model, unsigned numberOfPrincipalComponent) {
                return $self->BuildNewModelWithLeadingComponents(model, numberOfPrincipalComponent).release();
            }

            %newobject BuildNewModelWithVariance;
            StatisticalModelType* BuildNewModelWithVariance(const StatisticalModelType * model, double totalVariance) {
                return $self->BuildNewModelWithVariance(model, totalVariance).release();
            }
        }

        private:
            ReducedVarianceModelBuilder();
};

}

%template(ReducedVarianceModelBuilder_vtkPD) statismo::ReducedVarianceModelBuilder<vtkPolyData>;
%template(ReducedVarianceModelBuilder_vtkUG) statismo::ReducedVarianceModelBuilder<vtkUnstructuredGrid>;
%template(ReducedVarianceModelBuilder_vtkSP) statismo::ReducedVarianceModelBuilder<vtkStructuredPoints>;

//////////////////////////////////////////////////////
// IO
//////////////////////////////////////////////////////

namespace statismo {
template <typename T>
class IO {
    private:
        typedef StatisticalModel<T> StatisticalModelType;
        IO();
    public:

        // Overcome unique pointer problem
        %ignore IO::LoadStatisticalModel(typename StatisticalModelType::RepresenterType *, const std::string &, unsigned);
        %ignore IO::LoadStatisticalModel(typename StatisticalModelType::RepresenterType *, const H5::Group &, unsigned);
        %extend {
            %newobject LoadStatisticalModel;
            static StatisticalModelType* LoadStatisticalModel(
                                    Representer<T> *representer,
                                    const std::string &filename,
                                    unsigned maxNumberOfPCAComponents = std::numeric_limits<unsigned>::max()) {
                return statismo::IO<T>::LoadStatisticalModel(representer, filename, maxNumberOfPCAComponents).release();
            }

            %newobject LoadStatisticalModel;
            static StatisticalModelType* LoadStatisticalModel(
                                    Representer<T> *representer,
                                    const H5::Group &modelRoot,
                                    unsigned maxNumberOfPCAComponents = std::numeric_limits<unsigned>::max()) {
                return statismo::IO<T>::LoadStatisticalModel(representer, modelRoot, maxNumberOfPCAComponents).release();
            }
        }

        static void SaveStatisticalModel(const StatisticalModelType *const model, const std::string &filename);
};
}

%template(IO_vtkPD) statismo::IO<vtkPolyData>;
%template(IO_vtkUG) statismo::IO<vtkUnstructuredGrid>;
%template(IO_vtkSP) statismo::IO<vtkStructuredPoints>;
