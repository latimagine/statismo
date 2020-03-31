Modules Description
===================

Source code of the projects clustered into modules.

File Hierarchy
==============

~~~
├── core                                : Core module with basic features
│   └── include                         : Header files
│   └── src                             : Source files
│   └── tests                           : Test scripts
│   └── .clang-tidy                     : Code style and sanity configuration file
├── ITK                                 : ITK compatibility module
│   └── cli                             : cli tools scripts
│       └── tests                       : cli tools tests
│       └── *.md                        : cli tools documentation
│   └── examples                        : Examples scripts
│   └── include                         : Header files
│   └── tests                           : Test scripts
├── VTK                                 : VTK compatibility module with python wrapping
│   └── examples                        : Examples scripts
│   └── include                         : Header files
│   └── src                             : Source files
│   └── tests                           : Test scripts
│   └── wrapping                        : Python wrapper directory
│       └── tests                       : Python wrapper tests scripts
│       └── *.i                         : Swig interface descriptors
│       └── requirements.txt            : Requirements for using the wrapper
│           └── requirements_tests.txt  : Requirements for running python wrapper unit tests
│   └── .clang-tidy                     : Code style and sanity configuration file
~~~

Note on Examples
================

The ITK and VTK examples are simple examples showing the use of statismo.

See the ```run*Examples.sh``` scripts to see how to use the example programs or to run them directly (on Unix).

All the example are layed out all-in-one meaning you can copy paste it to have a working example at the expense of some code reuse of course!

The code standard (error handling, coding style and performance) for these examples
is lower than the true user utilities found in cli. These examples can be seen as code snaphshots that can be run!
