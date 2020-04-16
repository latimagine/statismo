[![Latest Release](https://img.shields.io/badge/release-statismo%2F0.11.0-blue.svg)](https://github.com/kenavolic/statismo/releases)
[![Build Status](https://api.travis-ci.org/kenavolic/statismo.svg?branch=develop)](https://travis-ci.org/kenavolic/statismo)
[![Documentation](https://img.shields.io/badge/docs-doxygen-blue.svg)](http://kenavolic.github.io/statismo/)
[![License](https://img.shields.io/badge/license-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

Statismo
========

Statismo is a c++ framework for statistical shape modeling. It supports all shape modeling tasks, from model building to shape analysis. Although the main focus of statismo lies on shape modeling, it is designed such that it supports any kind of PCA based statistical model, including statistical deformation models and intensiy models. One of the main goals of statismo is to make the exchange of statistical shape models easy. This is achieved by using a well documented file format based on [HDF5](https://www.hdfgroup.org). It also add shape modeling capabilities to other frameworks used in medical imaging ([VTK](https://vtk.org/) and [ITK](https://itk.org/)).

This repository is a fork of the [original projet](https://github.com/statismo/statismo) created for keeping the SSM features available to the C++ community now that the original
team is more focused on the development and maintainance of [Scalismo](https://github.com/unibas-gravis/scalismo).

The main purposes of this fork are:
* Keeping the framework up-to-date with new build toochain versions (cmake, compilers),
* Keeping the framework up-to-date with new language standards (c++, python),
* Keeping the framework compatible with new dependency versions,
* Backport main compatible features from Scalismo.

Browse the [About section](doc/md/ABOUT.md) to get more information about the project (history, details).

Quick Start
===========

Installation instructions can be found in the [Install section](doc/md/INSTALL.md).

Usage instructions can be found in the [Usage section](doc/md/USE.md).

Need Some Help?
===============

Try the following locations to get more documentation on the project:
* [Install section](doc/md/INSTALL.md),
* [Usage section](doc/md/USE.md),
* [API Doxygen doc](http://kenavolic.github.io/statismo/),
* [Related presentations](doc/md/ABOUT.md#Presentations),
* [Related publications](doc/md/ABOUT.md#Scientific-Publications).

In the source code, each directory contains a README that can help you understand the file hierarchy.

There is also the [statismo-users](https://groups.google.com/forum/#!forum/statismo-users) google group for general questions and discussion regarding statismo and shape models in general.

Do not hesitate to browse the project [issues](https://github.com/kenavolic/statismo/issues) or read the [Known Build Issues](doc/md/INSTALL.md#Known-Build-Issues) if you encountered some problems with the project.

Want To Help?
=============

Any help would be greatly appreciate as the current resources dedicated
to the project are limited.

Read up on our guidelines for [contributing](CONTRIBUTING.md)!

You can file a bug or check out for an issue to solve [here](https://github.com/kenavolic/statismo/issues).

You can also contribute to new features described in the [TODO list](doc/md/TODO.md). Specific contributions to the [project compatiblity](doc/md/TODO.md#Compatibility) or the [packaging](doc/md/TODO.md#Packaging) would be very useful.

Any other code contribution or documentation improvement would be excellent too, of course!

Do not hesitate to keep the community alive by sharing your questions and experiences on the [group](https://groups.google.com/forum/#!forum/statismo-users).

Version Numbers
===============

Statismo uses semantic versioning based on a sequence of identifiers (*x.y.z*) that are updated according to change significance and degree of compatility:
* *x* is the major version (increment for changes which are not backward-compatible),
* *y* is the minor version (increment for new but backward-compatible API features),
* *z* is the patch version (increment for fixes that does not change the API).

WARNING: Before the first *1.y.z* version, backward-compatibility is not ensured.

License
=======

Statismo itself is licensed under the BSD license. It depends, however, on other open source projects, which are distributed under different licenses. Most notably, these are: 
* [Eigen](http://eigen.tuxfamily.org),
* [HDF5](http://www.hdfgroup.org).

and, depending on the configuration:
* [ITK](http://www.itk.org),
* [VTK](http://www.vtk.org).


Acknowledgement
===============

Thanks to the [team](doc/md/ABOUT.md#History) behind [Statismo](https://github.com/statismo/statismo) and [Scalismo](https://github.com/unibas-gravis/scalismo) for their great work.

Thanks to [Marcel Luethi](https://github.com/marcelluethi) for his sound advises.
