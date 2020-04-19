# -*- coding: utf-8 -*-

import glob
import os
from pathlib import Path

from conans import CMake, ConanFile, tools


class LibItkConan(ConanFile):
    name = "itk"
    version = "5.0.1"
    short_version = '.'.join(version.split('.')[:2])
    description = "Toolkit for N-dimensional scientific image processing, segmentation, and registration"
    homepage = "https://itk.org/"
    url = "https://github.com/InsightSoftwareConsortium/ITK"
    license = "Apache 2.0"
    exports_sources = ["CMakeLists.txt"]
    exports = ["FindITK.cmake"]
    generators = "cmake", "cmake_paths", "cmake_find_package_multi"

    settings = "os", "arch", "compiler", "build_type"
    options = {
        "shared": [True, False],
        "fPIC": [True, False],
        # TODO: support more modules as required by client projects
        "module_itkvtkglue": [True, False]
    }
    default_options = {
        "shared": True,
        "fPIC": True,
        "module_itkvtkglue": True
    }

    _source_subfolder = "source_subfolder"
    _build_subfolder = "build_subfolder"
    _cmake = None

    scm = {
        "type": "git",
        "subfolder": _source_subfolder,
        "url": "https://github.com/InsightSoftwareConsortium/ITK.git",
        "revision": "v{}".format(version)
    }

    def config_options(self):
        if self.settings.compiler == "Visual Studio":
            del self.options.fPIC

    def configure(self):
        self.options["hdf5"].shared = self.options.shared

    def source(self):
        # Use Eigen3::Eigen3 target defined by Conan
        tools.replace_in_file(
            # Convert Path to str for compatibility with python 3.5
            str(Path(self._source_subfolder) / "Modules" / \
                "ThirdParty" / "Eigen3" / "CMakeLists.txt"),
            'Eigen3::Eigen',
            'Eigen3::Eigen3'
        )
        # Use HDF5::HDF5 target defined by Conan
        tools.replace_in_file(
            # Convert Path to str for compatibility with python 3.5
            str(Path(self._source_subfolder) / "Modules" / \
                "ThirdParty" / "HDF5" / "CMakeLists.txt"),
            'set(ITKHDF5_LIBRARIES )',
            'set(ITKHDF5_LIBRARIES HDF5::HDF5)'
        )

    def requirements(self):
        self.requires("double-conversion/3.1.5")
        self.requires("eigen/3.3.7")
        self.requires("hdf5/1.10.6")
        self.requires("libjpeg-turbo/2.0.4")
        self.requires("libpng/1.6.37")
        self.requires("libtiff/4.0.9@bincrafters/stable")
        self.requires("zlib/1.2.11")

        if self.options.module_itkvtkglue:
            self.requires("vtk/8.2.0@user/stable")

    def _configure_cmake(self):
        if self._cmake is not None:
            return self._cmake
        self._cmake = CMake(self)

        # Add this to include package root as module paths (needed for proper VTK inclusion)
        self._cmake.definitions["CMAKE_TOOLCHAIN_FILE"] = "conan_paths.cmake"

        self._cmake.definitions["BUILD_SHARED_LIBS"] = self.options.shared

        # Build only necessary modules
        self._cmake.definitions["BUILD_DOCUMENTATION"] = "OFF"
        self._cmake.definitions["BUILD_EXAMPLES"] = "OFF"
        self._cmake.definitions["BUILD_TESTING"] = "OFF"

        # TODO: Update when more options will be added
        self._cmake.definitions["Module_ITKVtkGlue"] = self.options.module_itkvtkglue

        # Tell CMake to find system (Conan) modules
        # TODO: Add libraries here when available on a remote (conan-center etc.)
        self._cmake.definitions["ITK_USE_SYSTEM_DOUBLECONVERSION"] = "ON"
        self._cmake.definitions["ITK_USE_SYSTEM_EIGEN"] = "ON"
        self._cmake.definitions["ITK_USE_SYSTEM_EXPAT"] = "ON"
        self._cmake.definitions["ITK_USE_SYSTEM_FFTW"] = "ON"
        self._cmake.definitions["ITK_USE_SYSTEM_HDF5"] = "ON"
        self._cmake.definitions["ITK_USE_SYSTEM_JPEG"] = "ON"
        self._cmake.definitions["ITK_USE_SYSTEM_PNG"] = "ON"
        self._cmake.definitions["ITK_USE_SYSTEM_TIFF"] = "ON"
        self._cmake.definitions["ITK_USE_SYSTEM_ZLIB"] = "ON"

        self._cmake.configure(build_folder=self._build_subfolder)
        return self._cmake

    def build(self):
        cmake = self._configure_cmake()
        if self.settings.os == 'Macos':
            # run_environment does not work here because it appends path just from
            # requirements, not from this package itself
            # https://docs.conan.io/en/latest/reference/build_helpers/run_environment.html#runenvironment
            lib_path = os.path.join(self.build_folder, 'lib')
            self.run(
                'DYLD_LIBRARY_PATH={0} cmake --build . {1}'.format(lib_path, cmake.build_config))
        else:
            cmake.build()

    def package(self):
        cmake = self._configure_cmake()
        cmake.install()
        cmake.patch_config_paths()

        # conan cmake_paths generator adds package root in cmake module path, thus
        # we copy the find package there
        self.copy("FindITK.cmake", keep_path=False)

    def package_info(self):
        self.cpp_info.names["cmake_find_package"] = "ITK"
        self.cpp_info.names["cmake_find_package_multi"] = "ITK"
        self.cpp_info.libs = tools.collect_libs(self)
        self.cpp_info.includedirs = [
            "include/ITK-{}".format(self.short_version)]

        if self.settings.os == "Linux":
            self.cpp_info.system_libs.append("pthread")
