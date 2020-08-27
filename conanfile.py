# -*- coding: utf-8 -*-

import os
from pathlib import Path
from packaging import version

from conans import ConanFile, CMake, tools, RunEnvironment
from conans.errors import ConanInvalidConfiguration


class LibStatismoConan(ConanFile):
    name = "statismo"
    version = "0.12.1"
    version_split = version.split('.')
    description = "Framework for building Statistical Image And Shape Models"
    url = "https://github.com/kenavolic/statismo"
    license = "BSD-3-clause"
    exports_sources = "*"
    generators = "cmake", "cmake_paths", "cmake_find_package_multi"

    settings = "os", "arch", "compiler", "build_type"
    options = {
        "shared": [True, False],
        "fPIC": [True, False],
        "with_tests": [True, False],
        "with_logs": [True, False]
        # TODO: support more options as required by client projects
    }
    default_options = {
        "shared": True,
        "fPIC": True,
        "with_tests": True,
        "with_logs": False
        # TODO: support more options as required by client projects
    }

    _build_subfolder = "build_subfolder"
    _cmake = None

    def config_options(self):
        if self.settings.compiler == "Visual Studio":
            del self.options.fPIC

    def configure(self):
        if self.settings.compiler == 'gcc' and self.settings.compiler.libcxx != 'libstdc++11':
            raise ConanInvalidConfiguration(
                "Statismo requires ABI 'libstdc++11' when built with 'gcc'")

        tools.check_min_cppstd(self, "14")

        self.options["hdf5"].shared = self.options.shared

    def source(self):
        # No-op for now
        pass

    def requirements(self):
        self.requires("cmake/3.16.3")
        self.requires("eigen/3.3.7")
        self.requires("hdf5/1.10.6")
        self.requires("itk/5.0.1@user/stable")
        self.requires("vtk/8.2.0@user/stable")

    def _configure_cmake(self):
        if self._cmake is not None:
            return self._cmake
        self._cmake = CMake(self)

        # Add this to include package root as module paths (needed for proper VTK/ITK inclusion)
        self._cmake.definitions["CMAKE_TOOLCHAIN_FILE"] = "conan_paths.cmake"

        # Build only necessary modules
        self._cmake.definitions["BUILD_DOCUMENTATION"] = "OFF"
        self._cmake.definitions["BUILD_TESTS"] = self.options.with_tests
        self._cmake.definitions["BUILD_TESTING"] = self.options.with_tests
        self._cmake.definitions["BUILD_EXAMPLES"] = "ON"
        self._cmake.definitions["USE_ITK_EIGEN"] = "OFF"
        self._cmake.definitions["USE_ITK_HDF5"] = "OFF"
        self._cmake.definitions["BUILD_SHARED_LIBS"] = self.options.shared
        self._cmake.definitions["ENABLE_RUNTIME_LOGS"] = self.options.with_logs
        # TODO: Update when more options will be added

        if self.settings.os == "Macos":
            self.env["DYLD_LIBRARY_PATH"] = os.path.join(
                self.build_folder, "lib")
            self.output.info("cmake build: %s" % self.build_folder)

        self._cmake.configure(build_folder=self._build_subfolder)
        return self._cmake

    def build(self):
        cmake = self._configure_cmake()

        environ_backup = os.environ.copy()
        os.environ['CONAN_CPU_COUNT'] = '2'
        try:
            if self.settings.os == "Macos":
                # run_environment does not work here because it appends path just from
                # requirements, not from this package itself
                # https://docs.conan.io/en/latest/reference/build_helpers/run_environment.html#runenvironment
                lib_path = os.path.join(self.build_folder, "lib")
                self.run(
                    "DYLD_LIBRARY_PATH={0} cmake --build . {1}".format(lib_path, cmake.build_config))
            else:
                cmake.build()
        finally:
            os.environ.clear()
            os.environ.update(environ_backup)

        if self.options.with_tests:
            env_build = RunEnvironment(self)
            with tools.environment_append(env_build.vars):
                cmake.parallel = False
                cmake.verbose = True
                cmake.test(output_on_failure=True)

    def package(self):
        cmake = self._configure_cmake()
        cmake.install()
        cmake.patch_config_paths()

    def package_info(self):
        self.cpp_info.names["cmake_find_package"] = "statismo"
        self.cpp_info.names["cmake_find_package_multi"] = "statismo"
        self.cpp_info.libs = tools.collect_libs(self)

        self.cpp_info.includedirs = [
            "include"
        ]
