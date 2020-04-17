Don't Want To Code?
===================

> :information_source: To use statismo-based tools on Windows with dependencies generated in DLL
> format, it is necessary to update the path accordingly:
```
set PATH=C:\path\to\vtk\bin;C:\path\to\itk\bin;C:\path\to\hdf5\bin;%PATH%
```

> :information_source: On Linux, if some dependencies are installed as shared libs in 
> non-standard locations, it is necessary to inform the system about it:
```
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/src/hdf5/dist/lib:/usr/src/itk/dist/lib:/usr/src/vtk/dist/lib
```

CLI tools
---------

> :information_source: To use the cli tools, you need a version of Statismo compiled with **BUILD_CLI_TOOLS=ON**.

### Tools List

| Tool                              | Desc                                                       | Doc link        |
|-----------------------------------|------------------------------------------------------------|-----------------|
|statismo-build-deformation-model   |builds deformation models from a list of deformation fields |[link](../../modules/ITK/cli/statismo-build-deformation-model.md) |
|statismo-build-gp-model   |builds shape or deformation models from a gaussian process definition |[link](../../modules/ITK/cli/statismo-build-gp-model.md) |
|statismo-build-shape-model   |builds shape models from meshes |[link](../../modules/ITK/cli/statismo-build-shape-model.md) |
|statismo-fit-image    |fits a model iteratively to an image |[link](../../modules/ITK/cli/statismo-fit-image.md) |
|statismo-fit-surface   |fits a model iteratively in to a target mesh |[link](../../modules/ITK/cli/statismo-fit-surface.md) |
|statismo-posterior   |creates a posterior model from an existing model |[link](../../modules/ITK/cli/statismo-posterior.md) |
|statismo-reduce-model   |reduces the number of components in a model |[link](../../modules/ITK/cli/statismo-reduce-model.md) |
|statismo-sample   |draws samples from a model |[link](../../modules/ITK/cli/statismo-sample.md) |
|statismo-warp-image   |applies a deformation field to an image |[link](../../modules/ITK/cli/statismo-warp-image.md) |

The use cases will show you usages example as well as the cli test script located in *modules/ITK/cli/tests*.

### Use Case: Shape Model Building

#### Prerequisites

* Surfaces aligned (i.e. there is no global rotation/translation).

#### Gaussian Process Model Building

Build a gaussian process model from a reference shape:
```
> statismo-build-gp-model -r surfaces/reference.vtk -k gaussian -s 50 -p 50 -n 200 gaussmodel.h5
```

#### Shape Sampling

Generate mean shape:
```
> statismo-sample -i gaussmodel.h5 -m -o mean.vtk
```

Generate random shapes:
```
> statismo-sample -i gaussmodel.h5 -o randomsample.vtk
```

Generate random shapes along a principal axis (here first component is 3 stddev from mean):
```
> statismo-sample -i gaussmodel.h5  -p 1:+3  -o pc1p3.vtk
```

#### Point-to-point Correspondences

```
> statismo-fit-surface -i gaussmodel.h5 -t surfaces/mesh2.vtk -w 0.1 -p -o fit-mesh2.vtk -j fit-projected-mesh2.vtk
```

Look at the fitting result. If there are still some differences, either:
 * improve the model (e.g. modify parameters) or,
 * add known correspondences as landmarks
```
> statismo-fit-surface -v 2.0 -f landmarks/reference-lm.csv -m landmarks/mesh2-lm.csv -i gaussmodel.h5 -t surfaces/mesh2.vtk -w 0.0001 -p -o fit-mesh2.vtk -j fit-projected-mesh2.vtk
```

This procedure build internally a posterior model, i.e. a shape model which is constrained to pass through the landmark points. We can also construct this model explicitly using:
```
> statismo-posterior -i gaussmodel.h5 -f reference-lm.csv -m mesh2-lm.csv -v 2.0 -o gpposterior.h5
```

And use it as an input for the fitting procedure:
```
> statismo-fit-surface -i gpposterior.h5 -t surfaces/mesh2.vtk -w 0.1 -p -o fit-mesh2.vtk -j fit-projected-mesh2.vtk
```

#### PCA Model Building

The point-to-point correspondence step must be performed for all meshes in the dataset. This will
imply that all output meshes have the same number of points as the reference, and denote the "same" anatomical point on the surface.

```
> statismo-build-shape-model -p GPA -l datalist.txt -o pcamodel.h5
```
The file ```datalist.txt``` is a text file that contains the filenames of all the examples used for model building (one filename per line). 

#### Model Flexibility Enhancement

The idea is to improve the expressiveness of a model by adding more knowledge to a model
built with insufficient data.
```
> statismo-build-gp-model -m pcamodel.h5 -k gaussian -s 5 -p 200 -n 50 -o expressivepcamodel.h5
```

In this case we specified that we would like to add smooth deformations, modelled by a Gaussian kernel with standard deviation 200 and average scale of 5mm, to our model. This corresponds to the assumption that the bias of the PCA model (i.e. the shape variations that could not be explained by the training data) can be modelled by small and very smoothly varying deformations. We choose to represent the final model using 50 basis functions (-n 50). This parameter should be chosen such that it is larger than the number of principal components of the original model.

### Use Case: Deformation Models

> :information_source: The data used in this use case is available in the [repo](https://github.com/kenavolic/statismo/tree/master/data).

#### Deformation Model Building

* Learn a model from a deformation field, or
```
> statismo-build-deformation-model -l datalist.txt -d 2 -o deformation-model.h5
```
The file ```datalist.txt``` is a text file that contains the filenames of the deformation fields (one filename per line).
* Specifying the deformation using a Gaussian process
```
> statismo-build-gp-model -t deformation -d 2 -k gaussian -p 50 -s 50 -n 200 -r df-hand-1.vtk -o deformation-model.h5
```

#### Deformation Field Sampling

```
> statismo-sample -t deformation -d 2 -i deformation-model.h5 -o df-sample.vtk
```

```
> statismo-warp-image -d 2 -i hand_images/hand-0.vtk -f df-sample.vtk -o warped-image.vtk
```

#### Registration

The, we can use the model to fit the model to an image:

```
> statismo-fit-image -d 2 -p -i deformation-model.h5 -f hand_images/hand-0.vtk -m hand_images/hand-1.vtk -w 0.1 -o registered-image.vtk -e deformation-field.vtk
```

Associated tools
----------------

There is a project to enable the use of [Scalismo Viewer](https://github.com/unibas-gravis/scalismo-ui) with Statismo C++ framework.

To look at the content of a model HDF5 file, you can use [Hdfview](http://www.hdfgroup.org/products/java/hdfview/).

Want To Code (or Must)?
=======================

Quick prototyping with python
-----------------------------

> :information_source: For now, you can just use the VTK module with python.

> :information_source: To use python wrapping, you need a version of Statismo compiled with wrapping enabled (see [instructions here](INSTALL.md#Build-Wrapping)).

The easiest way to use python is to use Docker as described [here](INSTALL.md#Docker).

Supposed you have a compatible version of Statismo installed, the following
commands should help you for setting up a new environment:
```
> virtualenv -p /usr/bin/python3 statismo-venv
> source statismo-venv/bin/activate
(statismo-venv)> python -m pip install --upgrade pip
# The requirements defined in modules/VTK/wrapping
(statismo-venv)> pip install -r requirements.txt
(statismo-venv)> export PYTHONPATH=$PYTHONPATH:/path/to/vtk/install/dir/lib/python3.5/site-packages:/path/to/statismo/install/dir/lib/python3.5/site-packages
(statismo-venv)> export LD_LIBRARY_PATH=/path/to/vtk/install/dir/lib:/path/to/statismo/install/dir/lib:$LD_LIBRARY_PATH
```

> :information_source: At the root of Statismo build directory, you can
> find a script called ```runVTKPythonTestsRelease.sh``` that is used in CI
> to run the tests

> :information_source: If Statismo is compiled with **USE_SYSTEM_VTK=OFF**,
> the default INSTALL directory for VTK is ```${statismobuildir}/INSTALL```

Once the environment is setup, just play with Statismo and VTK:
```
> python
>>> import vtk
>>> import statismovtk
>>> builder = statismovtk.PCAModelBuilder_vtkPD.Create()
```

Usage examples can be found in the [unit tests scripts](https://github.com/kenavolic/statismo/tree/master/modules/VTK/wrapping/tests/statismoTests).


Build an Application upon Statismo
----------------------------------

### CMake App

> :information_source: The prerequisite is that Statismo is installed
> in a directory *${statismo_install_dir}*

The minimal CMakeLists.txt of an application should look like this:
```
# Minimum CMake required
cmake_minimum_required(VERSION 3.15)

# Project
project(statismo-app LANGUAGES C CXX VERSION 0.1.0)

# External dependencies
find_package(statismo REQUIRED)
include(${STATISMO_USE_FILE})

add_executable(statismo-app main.cpp)
target_link_libraries(statismo-app ${STATISMO_LIBRARIES} ${VTK_LIBRARIES} ${ITK_LIBRARIES})
```

To configure it to use the installed Statismo on Unix system:
```
> cmake ${statismo-app-root} -Dstatismo_DIR=${statismo_install_dir}/lib/cmake/statismo/
```

To configure it to use the installed Statismo on Windows system:
```
> cmake ${statismo-app-root} -Dstatismo_DIR=${statismo_install_dir}/CMake/
```

> :warning: On Linux and Windows with shared libraries, you might have to inform
> the system of paths to statismo and/or its dependencies if it is in an unusal
> location (or all the time under Windows) by updating ```$LD_LIBRARY_PATH or %PATH%```

```
set PATH=%vtk_install_dir%\dist\bin;%itk_install_dir%\bin;hdf5_install_dir\bin;%PATH%
```
> :information_source: Do not hesitate to browse the installation sample
> scripts described [here](INSTALL.md#Configuration-Examples). It can guide
> you as often, a demo app is installed in the validation process.

### CMake/Conan App

> :information_source: The prerequisite is that Statismo was installed
> with conan package manager as described [here](INSTALL.md#Conan-(Experimental)).

An example layout could be:
~~~
├── CMakeLists.txt  : CMake main script
├── conanfile.txt   : Conan description file
├── main.cpp        : Executable file
~~~

*CMakeLists.txt*:
~~~
cmake_minimum_required(VERSION 3.5)
project(Demo VERSION 0.1.0 LANGUAGES CXX)

# Setup dependencies via Conan
if (NOT EXISTS "${CMAKE_BINARY_DIR}/conan.cmake")
    message(STATUS "Downloading conan.cmake from https://github.com/conan-io/cmake-conan")
    file(DOWNLOAD "https://github.com/conan-io/cmake-conan/raw/v0.15/conan.cmake"
                  "${CMAKE_BINARY_DIR}/conan.cmake")
endif()
include(${CMAKE_BINARY_DIR}/conan.cmake)

conan_cmake_run(CONANFILE conanfile.txt
                BASIC_SETUP
                BUILD missing)
include(${CMAKE_BINARY_DIR}/conan_paths.cmake)

# Create an executable
find_package(statismo CONFIG REQUIRED)
include(${STATISMO_USE_FILE})

add_executable(${PROJECT_NAME}
    main.cpp
)
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)

target_link_libraries(${PROJECT_NAME} PRIVATE
    ${CONAN_LIBS}
)
~~~

*main.cpp*:
~~~
#include "statismo/VTK/vtkStandardMeshRepresenter.h"
#include "statismo/ITK/itkStandardMeshRepresenter.h"
#include <iostream>

int main() {
    std::cout << "DEMO: Creating an ITK representer" << std::endl;
    auto itkrep = itk::StandardMeshRepresenter<float, 3>::New();
    std::cout << "DEMO: ITK representer created at " << itkrep.GetPointer() << std::endl;
    
    std::cout << "DEMO: Creating a VTK representer" << std::endl;
    auto vtkrep = statismo::vtkStandardMeshRepresenter::SafeCreate();
    std::cout << "DEMO: VTK representer created at " << vtkrep.get() << std::endl;

    return EXIT_SUCCESS;
}
~~~

*conanfile.txt*:
~~~
[requires]
statismo/0.12.0@user/stable
itk/5.0.1@user/stable
vtk/8.2.0@user/stable

[options]
*:shared=True

[generators]
cmake_find_package_multi
cmake_paths

[imports]
bin, *.dll -> ./bin
lib, *.dylib* -> ./bin
lib, *.so* -> ./bin
~~~

To compile and run the application:
~~~
(statismo-conan-venv)> cmake -H. -Bbuild -DCMAKE_BUILD_TYPE=Release
(statismo-conan-venv)> cd build
(statismo-conan-venv)> cmake --build . --config Release
(statismo-conan-venv)> export LD_LIBRARY_PATH=./bin
(statismo-conan-venv)> ./bin/Demo
~~~

### Error Handling

Statismo uses exception as error handling system. It might not be convenient to you.
You can use the internal exception translator if needed to transform it to
a c++ returned object.

```
#include "statismo/core/Exceptions.h"
// ...
auto res = statismo::Translate([]() {
    return statismo::DoSomethingThatThrow();
  });
```

### Logging

> :information_source: The prerequisite is that Statismo is configured
> with *ENABLE_RUNTIME_LOGS=ON*

The framework provides a logger implementation that enables logging in background and adding multiple log handlers to handle each log.

A log handler consists of a formatter to format the log and an output
writer to write the log somewhere.

The following formatters are provided:
* Statismo default formatter,
* ITK formatter,
* VTK formatter.

The following writers are provided:
* Standard output writer,
* File writer,
* ITK output window writer,
* VTK output window writer.

Moreover, two classes are provided to redirect VTK and ITK logs to Statismo logger.

The following section will direct you towards examples where the logger
is used.

### Examples list

The following tables will point you towards C++ examples developed in the project.

VTK based examples:

| Description                              | Link        |
|------------------------------------------|-------------|
| Sampling a model                         |[link](../../modules/VTK/examples/vtkBasicSamplingExample.cxx)|
| Build a conditional model                |[link](../../modules/VTK/examples/vtkBuildConditionalModelExample.cxx)|
| Build a gaussian process shape model     |[link](../../modules/VTK/examples/vtkBuildGaussianProcessShapeModelExample.cxx)|
| Build an intensity model                 |[link](../../modules/VTK/examples/vtkBuildIntensityModelExample.cxx)|
| Build a posterior model                  |[link](../../modules/VTK/examples/vtkBuildPosteriorModelExample.cxx)|
| Build a PCA shape model                  |[link](../../modules/VTK/examples/vtkBuildShapeModelExample.cxx)|
| Cross validation example                 |[link](../../modules/VTK/examples/vtkCrossValidationExample.cxx)|
| Build model with reduced variance        |[link](../../modules/VTK/examples/vtkReduceModelVarianceExample.cxx)|
| Spatially varying GP model               |[link](../../modules/VTK/examples/vtkSpatiallyVaryingGPModelExample.cxx)|
| Redirect log to VTK output window        |[link](../../modules/VTK/examples/vtkBasicSamplingExample.cxx)|
| Redirect VTK log to Statismo logger      |[link](../../modules/VTK/examples/vtkBuildPosteriorModelExample.cxx)|


ITK based examples:

| Description                              | Link        |
|------------------------------------------|-------------|
| Build a deformation model                |[link](../../modules/ITK/examples/itkBuildDeformationModel.cxx)|
| Build a gaussian process deformation model |[link](../../modules/ITK/examples/itkBuildGaussianProcessDeformationModel.cxx)|
| Build a shape model                      |[link](../../modules/ITK/examples/itkBuildShapeModel.cxx)|
| Build a shape model with reduced variance |[link](../../modules/ITK/examples/itkBuildShapeModel_75pcvar.cxx)|
| Deformation fitting                      |[link](../../modules/ITK/examples/itkDeformationModelFitting.cxx)|
| Landmarks constraints shape model fitting |[link](../../modules/ITK/examples/itkLandmarkConstrainedShapeModelFitting.cxx)|
| Gaussian process registration            |[link](../../modules/ITK/examples/itkLowRankGaussianProcessImageToImageRegistration.cxx)|
| Shape model fitting                      |[link](../../modules/ITK/examples/itkShapeModelFitting.cxx)|
| Gaussian process simple registration     |[link](../../modules/ITK/examples/itkSimpleGaussianProcessImageToImageRegistration.cxx)|
| Redirect log to ITK output window        |[link](../../modules/ITK/examples/itkBuildDeformationModel.cxx)|
| Redirect ITK log to Statismo logger      |[link](../../modules/ITK/examples/itkBuildShapeModel.cxx)|