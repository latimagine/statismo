Description
===========

This directory contains deployment modules and scripts.

File Hierarchy
==============

~~~
├── docker
│   └── Dockerfile              : Docker recipe for standard installation on Linux with VTK python wrapping
│   └── Dockerfile-*            : List of docker recipes used for offline validation
├── pack
│   └── debian                  : Debian packaging (not maintained anymore, see #Packaging)
│   └── conan                   : Scripts for conan packaging
│       └── conan-recipes       : Local recipes not available in any remote artifactory repo
│       └── test_package        : Conan deployment tests for statismo
│       └── conan_install.sh    : Installation utility script
├── scripts
│   └── linux_offline_tests.sh  : Script used to run offline validation tests with docker container
│   └── win_offline_tests.sh    : Scripts used to run offline validation tests on a virtual machine
~~~

Packaging
=========

Debian packaging is not maintained anymore (see [Contributing](../README.md#Want-To-Help)).
