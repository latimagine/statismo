Roadmap
=======

As the resources dedicated to the project are subject to change according
to fundings and/or developers' workload, it is not possible to provide
a detailed roadmap with milestones here.

A list of features arranged thematically is provided instead. No priority
is assigned as it is also subject to change!

Wish List
=========

# Usability

* Make a python wrapper for the ITK module available as an external project
* Implement a bridge to make [Scalismo-UI](https://github.com/unibas-gravis/scalismo-ui) usable with Statismo

# Packaging

* Make Statismo available as a debian package again
* Make Statismo available through homebrew again
* Make Statismo VTK python wrapper available through pip
* Add a conan packaging layer to the framework

# Compatibility

* Extend OS compatibility
* Extend build toolchain compatibility
* Extend dependencies version compatibility
* Make the framework interfaces Dll-proof on Windows

# Modernization

* Use c++ 20 features
* Update ITK module to use modern C++ to loop in ITK data structure

# Sanity

* Add clang-tidy to CI when more CI resources will be available (free travis account for now)
* Add more configuration coverage to CI when more CI resources will be available (free travis account for now)

# Scalismo Backport

* Ability to go from discrete to continuous representation and back by means of interpolation
* [Pivoted Cholesky computation](https://github.com/unibas-gravis/scalismo/blob/master/src/main/scala/scalismo/numerics/PivotedCholesky.scala)

# Features

* Add ITK PointSet representer
