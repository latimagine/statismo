# -*- coding: utf-8 -*-

import glob
import os
from pathlib import Path

from conans import ConanFile, CMake, tools


class LibVTKConan(ConanFile):
    name = "vtk"
    version = "8.2.0"
    version_split = version.split('.')
    short_version = "%s.%s" % (version_split[0], version_split[1])
    description = "The Visualization Toolkit (VTK) is an open-source, \
        freely available software system for 3D computer graphics, \
        image processing, and visualization."
    homepage = "https://www.vtk.org/"
    url = "https://gitlab.kitware.com/vtk/vtk"
    license = "BSD license"
    exports_sources = ["CMakeLists.txt"]
    exports = ["FindVTK.cmake"]
    generators = "cmake"

    settings = "os", "arch", "compiler", "build_type"
    options = {
        "shared": [True, False],
        "fPIC": [True, False],
        # TODO: support more modules as required by client projects
        "qt": [True, False]
    }
    default_options = {
        "shared": True,
        "fPIC": True,
        # TODO: support more modules as required by client projects
        "qt": False
    }

    _source_subfolder = "source_subfolder"
    _build_subfolder = "build_subfolder"
    _cmake = None

    scm = {
        "type": "git",
        "subfolder": _source_subfolder,
        "url": "https://github.com/Kitware/VTK.git",
        "revision": "v{}".format(version)
    }

    def config_options(self):
        if self.settings.compiler == "Visual Studio":
            del self.options.fPIC

    def configure(self):
        # Solve spurious undefined reference issues
        self.options["ogg"].shared = True
        self.options["libxml2"].shared = True

        self.options["hdf5"].shared = self.options.shared

        # Configure Qt options
        if self.options.qt:
            self.options["qt"].shared = True
            self.options["qt"].qtx11extras = True

    def requirements(self):
        self.requires("double-conversion/3.1.5")
        self.requires("eigen/3.3.7")
        self.requires("expat/2.2.9")
        self.requires("freetype/2.10.1")
        self.requires("hdf5/1.10.6")
        self.requires("jsoncpp/1.9.2")
        self.requires("libjpeg-turbo/2.0.4")
        self.requires("libpng/1.6.37")
        self.requires("libtiff/4.0.9@bincrafters/stable")
        self.requires("libxml2/2.9.9")
        self.requires("lz4/1.9.2")
        self.requires("ogg/1.3.4")
        self.requires("pugixml/1.10@bincrafters/stable")
        self.requires("sqlite3/3.31.0")
        self.requires("theora/1.1.1@bincrafters/stable")
        self.requires("xz_utils/5.2.4")
        self.requires("zlib/1.2.11")
        self.requires("zstd/1.4.4", override=True)

        if self.options.qt:
            self.requires("qt/5.14.1@bincrafters/stable")

    def system_requirements(self):
        pack_names = None
        if tools.os_info.is_linux:
            if tools.os_info.with_apt:
                pack_names = [
                    "freeglut3-dev",
                    "mesa-common-dev",
                    "mesa-utils-extra",
                    "libgl1-mesa-dev",
                    "libglapi-mesa",
                    "libsm-dev",
                    "libxext-dev",
                    "libxt-dev",
                    "libglu1-mesa-dev",
                    "libx11-dev"
                ]

        if pack_names:
            installer = tools.SystemPackageTool()
            for item in pack_names:
                installer.install(item)

    def _configure_cmake(self):
        if self._cmake is not None:
            return self._cmake
        self._cmake = CMake(self)

        self._cmake.definitions["BUILD_SHARED_LIBS"] = self.options.shared

        # Build only necessary modules
        self._cmake.definitions["BUILD_TESTING"] = "OFF"
        self._cmake.definitions["BUILD_EXAMPLES"] = "OFF"
        self._cmake.definitions["BUILD_DOCUMENTATION"] = "OFF"

        # TODO: Update when more options will be added

        # Tell CMake to find system (Conan) modules
        # TODO: Add libraries here when available on a remote (conan-center etc.)
        for package_name in [
            "DOUBLECONVERSION",
            "EIGEN",
            "EXPAT",
            "FREETYPE",
            "HDF5",
            "JPEG",
            "JSONCPP",
            "LIBXML2",
            "LZ4",
            "LZMA",
            "OGG",
            "PNG",
            "PUGIXML",
            "SQLITE",
            "THEORA",
            "TIFF",
            "ZLIB"
        ]:
            self._cmake.definitions["VTK_USE_SYSTEM_{}".format(
                package_name)] = "ON"

        if self.options.qt:
            self._cmake.definitions["VTK_Group_Qt"] = "ON"
            self._cmake.definitions["VTK_QT_VERSION"] = "5"

        if self.settings.os == "Macos":
            self.env["DYLD_LIBRARY_PATH"] = os.path.join(
                self.build_folder, "lib")
            self.output.info("cmake build: %s" % self.build_folder)

        self._cmake.configure(build_folder=self._build_subfolder)
        return self._cmake

    def build(self):
        cmake = self._configure_cmake()
        if self.settings.os == "Macos":
            # run_environment does not work here because it appends path just from
            # requirements, not from this package itself
            # https://docs.conan.io/en/latest/reference/build_helpers/run_environment.html#runenvironment
            lib_path = os.path.join(self.build_folder, "lib")
            self.run(
                "DYLD_LIBRARY_PATH={0} cmake --build . {1}".format(lib_path, cmake.build_config))
        else:
            cmake.build()

    def package(self):
        cmake = self._configure_cmake()
        cmake.install()
        cmake.patch_config_paths()

        # conan cmake_paths generator adds package root in cmake module path, thus
        # we copy the find package there
        self.copy("FindVTK.cmake", keep_path=False)

    def package_info(self):
        self.cpp_info.names["cmake_find_package"] = "VTK"
        self.cpp_info.names["cmake_find_package_multi"] = "VTK"
        self.cpp_info.libs = tools.collect_libs(self)

        self.cpp_info.includedirs = [
            "include/vtk-%s" % self.short_version
        ]

        if self.settings.os == "Linux":
            self.cpp_info.system_libs.append("pthread")
