Compatibility
=============

The framework is tested against the following configurations (CI means it is only tested
through Travis thus with limited extend):

| OS                                | Compiler                        |
| ----------------------------------| --------------------------------|
| Windows 10 enterprise             | Visual studio 16 2019           |
| Windows Server, version 180 (CI)  | Visual Studio 2017              |
| Ubuntu 16.04                      | g++-7, clang++-6.0              |
| Ubuntu 18.04                      | g++-7, clang++-7                |
| macOS 10.14 (CI)                  | Xcode 11.3.1                    |

The install and run is tested with the following dependencies versions:

| Dependency/Tool                   | Version Range                   |
| ----------------------------------|---------------------------------|
| Python (for wrappers)             | 2.7, 3.7.5, 3.7.6               |
| Swig (for wrappers)               | 3.0 -> 4.1                      |
| CMake                             | >= 3.13.4                       |
| HDF5                              | 1.10.2 -> 1.10.6                |
| Eigen                             | 3.3.5 -> 3.3.7                  |
| VTK                               | 8.0.0 -> 8.2.0                  |
| ITK                               | 5.0 -> 5.0.1                    |

Packaging/deployment is tested against the following tools versions:

| Tool                              | Version Range                   |
| ----------------------------------|---------------------------------|
| Conan                             | 1.24                            |
| Docker                            | 18                              |

> :information_source: This tables are not exhaustive thus the project might work
> with other configurations... or not! If you need to work with older versions of ITK, VTK, HDF5, Eigen, you can fallback to the original project as well!

> :warning: Testing the framework over many platforms with all 
> possible configurations requires both time and
> material resources. We do our best to make the framework
> available on different platforms. Feel free to contribute
> if you want to extend the compatibility of the framework!

Build & Install
===============

Quick Start
-----------

> :information_source: Links to configuration examples are available [here](#Configuration-Examples)

### Superbuild

Superbuild is the preferred way to build Statismo from sources. The default configuration will download, configure and install the dependencies for you.

Here is a typical workflow to build and install the project:

```
> git clone git@github.com:kenavolic/statismo.git
> cd statismo
> mkdir build
> cd build
> cmake ../superbuild -DBUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/path/to/install/dir
> cmake --build . --config Release
> cd Statismo-build
> cmake --install . --config Release
```

### Conan (Experimental)

> :warning: The conan packaging layer is experimental for now
> and only available on Linux (tested on Ubuntu 18.04).

The first step is to install [conan](https://docs.conan.io/en/latest/). Prefer the
use of virtual environment here.

~~~
> virtualenv -p /usr/bin/python3 statismo-conan-venv
> source statismo-conan-venv/bin/activate
(statismo-conan-venv)> python -m pip install --upgrade pip
(statismo-conan-venv)> pip install conan --upgrade
(statismo-conan-venv)> bash statismo/deploy/pack/conan/conan_install.sh $statismo_source_dir
~~~

### Docker

A template [Dockerfile](../../deploy/docker/Dockerfile) can be used to install the project within a container.

Here it the workflow to build and run the image:

```
docker build . -t statismo-img
docker run -i -t statismo-img /bin/bash
```

Build Configuration
-------------------

### General

The main options with their usage are presented in the following table. See [Dependency Management](#Dependency-Management) for the options related to the installation of Eigen3, HDF5, VTK and ITK.

| Options                    | Usage                                |
| ---------------------------| -------------------------------------|
| ITK_SUPPORT                | Build ITK module                     |
| VTK_SUPPORT                | Build VTK module                     |
| BUILD_SHARED_LIBS          | Build shared or static libraries     |
| BUILD_WRAPPING             | Build python wrapping for VTK module |
| BUILD_CLI_TOOLS            | Build cli tools based on ITK module  |

> :information_source: Configuration examples are available [here](#Configuration-Examples)

### Dependency Management

The project with all supports enabled depends on:
* Eigen3
* HDF5
* VTK
* ITK

The problem is that ITK depends on Eigen3 and HDF5 as well and contains internal
version of these libraries.

The default and preferred way to build the framework with ITK and VTK support
enabled will download ITK and VTK and use ITK internal Eigen3 and HDF5 libraries
to minimize possible conflicts.

Here is the options matrix for dependencies management for VTK support:

| VTK_SUPPORT | USE_SYSTEM_VTK | Description                      |
|-------------|----------------|----------------------------------|
|OFF          |Disabled        |No VTK support                    |
|ON           |OFF             |VTK internal                      |
|ON           |ON              |VTK given to cmake                |


Here is the options matrix for dependencies management for ITK support:
| ITK_SUPPORT | USE_SYSTEM_ITK | USE_SYSTEM_HDF5 | USE_ITK_HDF5 | USE_SYSTEM_EIGEN | USE_ITK_EIGEN | Description        |
|-------------|----------------|-----------------|--------------|------------------|---------------|--------------------|
|OFF          |Disabled        |ON               |Disabled      |ON                |Disabled       |No ITK support. Eigen and HDF5 given to cmake.|
|OFF          |Disabled        |OFF              |Disabled      |OFF               |Disabled       |No ITK support. Eigen and HDF5 internal.|
|ON           |ON              |Disabled         |ON            |Disabled          |ON             |ITK support. ITK given to cmake. Internal ITK HDF5 and Eigen libraries used|
|ON           |ON              |Disabled         |OFF           |Disabled          |OFF            |ITK support. ITK given to cmake. HDF5 and Eigen given to cmake|
|ON           |OFF             |ON               |Disabled      |ON                |Disabled       |ITK support. ITK internal. HDF5 and Eigen given to cmake and used by ITK|
|ON           |OFF             |OFF              |Disabled      |OFF               |Disabled            |ITK support. ITK internal. Internal ITK HDF5 and Eigen libraries used|

***internal*** means internal version (which is eventually automatically downloaded from the web).

***given to cmake*** means the path to the library cmake config file is given to cmake. Here
are some examples of cmake configuration:
```
> cmake ../superbuild -DITK_SUPPORT=OFF -DVTK_SUPPORT=OFF -DUSE_SYSTEM_HDF5=ON -DUSE_SYSTEM_EIGEN=ON -DHDF5_DIR=/path/to/install/share/cmake/hdf5/ -DEigen3_DIR=/path/to/install/share/eigen3/cmake/
> cmake ../superbuild -DITK_SUPPORT=OFF -DVTK_SUPPORT=ON -DUSE_SYSTEM_VTK=ON -DVTK_DIR=/path/to/cmake/config/file
```
> :information_source: Eigen3 can also be configured by directly giving the path to the include directory with the cmake option -DEIGEN3_INCLUDE_DIR. However, it is recommended to import Eigen3 the cmake way for ITK.

> :information_source: Configuration examples are available [here](#Configuration-Examples)

### Dependencies Configuration

If you use system libraries, it is your responsability to configure them
in a compatible way. To see the requirements for each dependency, feel free to browse the *External-{lib}.cmake* file of the library in the superbuild directory.

If you choose the internal option for ITK and VTK (**USE_SYSTEM_ITK=OFF**, **USE_SYSTEM_ITK=OFF**), and your final application based upon Statismo needs ITK and VTK configured with specific options, you can use **ITK_EXTRA_OPTIONS** and **VTK_EXTRA_OPTIONS**
to specify these extra requirements when building the dependencies.

```
# Enable nifti support for ITK
> cmake ../superbuild -DITK_EXTRA_OPTIONS="-DModule_ITKIONIFTI=ON"
# Force python version for VTK
> cmake ../superbuild -DVTK_EXTRA_OPTIONS="-DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython2.7.so.1;-DPYTHON_INCLUDE_DIR=/usr/include/python2.7/"
# Opengl rendering for VTK
> cmake ../superbuild -DVTK_EXTRA_OPTIONS="-DModule_vtkRenderingVolumeOpenGL=ON"
```

This can be useful on Windows to skip the path length limitation for ITK:

```
> cmake ../superbuild -DITK_EXTRA_OPTIONS:STRING="-DITK_SKIP_PATH_LENGTH_CHECKS=1"
```

### Build Wrapping

Requirements:
* Python installed with pip (python3 preferred)
* Swig installed (>=3)
* Required python modules installed:
```
# Browse to the VTK module wrapping directory
> cd modules/VTK/wrapping
# Install requirements to use the module
> pip install -r requirements.txt
# Install requirements to run the unit tests
> pip install -r requirements_tests.txt
```
* If you use a system VTK version, following build options must be set:
  * **VTK_WRAP_PYTHON=ON**
  * **BUILD_SHARED_LIBS=ON**
  * **BUILD_TYPE=Release**.

CMake configuration:
 * **BUILD_WRAPPING=ON** 
 * **BUILD_SHARED_LIBS=ON**
 * **BUILD_TYPE=Release**.

> :warning: VTK and Statismo must point towards the same python version!

> :information_source: Configuration examples are available [here](#Configuration-Examples)

### Build Documentation

Requirements:
 * pandoc installed
 * doxygen installed
 * latex installed

CMake configuration:
 * **BUILD_DOCUMENTATION=ON** (for doxygen doc)
 * **BUILD_CLI_TOOLS_DOC=ON** (for cli tools doc)

Build command for doxygen documentation:
```
> make statismo_dox
```

### Configuration Examples

The following table points to scripts (mainly used for testing) that build Statismo with some given options on different platforms. It can give you tips about the installation of the environment and the framework.

Note that dependencies are installed with *apt* on Linux, *chocolatey* on Windows and *homebrew* on OSx.

| OS          | Config summary | Script path |
|-------------|----------------|------------ |
|Windows      |ITK system, VTK system, ITK Eigen, ITK HDF5, static libs, debug |[ps script](../../deploy/scripts/win_offline_tests.ps1)|
|Windows      |ITK internal, VTK internal, Eigen system, HDF5 system, shared libs, release, python wrapping|[ps script](../../deploy/scripts/win_offline_tests.ps1)|
|Mac          |ITK system, VTK system, ITK Eigen, ITK HDF5, shared libs, release, python wrapping|[ci script](../../.travis.yml)|
|Linux Bionic |HDF5 internal, Eigen system |[docker file](../../deploy/docker/Dockerfile-hdf5-dl-eigen-sys-shared-release)|
|Linux Bionic |HDF5 system, Eigen internal |[docker file](../../deploy/docker/Dockerfile-hdf5-sys-eigen-dl-static-release)|
|Linux Bionic |ITK internal, VTK internal, HDF5 system, Eigen system, doc generation |[docker file](../../deploy/docker/Dockerfile-hdf5-sys-eigen-sys-itk-dl-vtk-dl-static-debug-with-doc)|
|Linux Bionic |ITK system, HDF5 system, Eigen system |[docker file](../../deploy/docker/Dockerfile-hdf5-sys-eigen-sys-itk-sys-shared-release)|
|Linux Bionic |ITK internal, VTK internal, ITK HDF5, ITK Eigen |[docker file](../../deploy/docker/Dockerfile-itkhdf5-itkeigen-itk-dl-vtk-dl-shared-release-with-wrapping)|
|Linux Bionic |ITK system, ITK HDF5, ITK Eigen |[docker file](../../deploy/docker/Dockerfile-itkhdf5-itkeigen-itk-sys-static-debug)|
|Linux Bionic |Conan |[Docker file](../../deploy/docker/Dockerfile-conan)|

### Platform Details

#### Linux

See [build issues](#Known-Build-Issues) for possible problems on Linux.

#### Windows

> :warning: Prefer static linking (*BUILD_SHARED_LIBS=OFF*) as the framework
> public interfaces are not DLL proof!

See [build issues](#Known-Build-Issues) for possible problems on Windows.

#### OSX

See [build issues](#Known-Build-Issues) for possible problems on OSX.

### Known Build Issues

| OS          | Issue          | External Link | Solution |
|-------------|----------------|---------------|----------|
|Windows      |*BUILD_SHARED_LIBS* forced to off on Win32 causing wrong system hdf5 libs to be selected in ITK | [link](https://github.com/InsightSoftwareConsortium/ITK/issues/1658) |Apply the patch |
|Windows      |ITK compilation failure with VS2019 in Debug mode | [link](https://discourse.itk.org/t/itk-5-0-deformableregistration15-cxx/1948) | Compile ITK with *-DCMAKE_CXX_FLAGS='/FS'*|
|Windows      |ITK path too long | - | Compile Statismo with *-DITK_EXTRA_OPTIONS:STRING="-DITK_SKIP_PATH_LENGTH_CHECKS=1"* |
|All      |compiler internal error | - | This can be due to your system being short in RAM |

Check Installation
===================

Run the c++ tests to check if the installation succeeded:
```
> cd build/Statismo-build
> ctest -C Release
```

If python wrapping was enabled, run the python tests as well:
```
> cd build/Statismo-build
> sh ./runVTKPythonTestsRelease.sh
```
