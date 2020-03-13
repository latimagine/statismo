History
=======

Statismo has originally been developed in the context of the [Co-Me](http://www.co-me.ch) research project as a collaboration between the [University of Bern](http://www.istb.unibe.ch), the [University of Basel](http://gravis.cs.unibas.ch) and the [ETH Zurich](http://www.vision.ee.ethz.ch/), with goal of the making it easy to exchange algorithms and shape models between different research groups. The original code has been written by:
* Marcel Luethi, University of Basel and
* Remi Blanc, formerly at ETH Zurich.

In the meantime, many people have contributed to statismo, including:
- Thomas Albrecht
- Tobias Gass
- Arnaud Gelas
- Thomas Gerig
- Christoph Jud
- Christoph Langguth
- Frank Mueller
- Stefan Schlager
- Sandro Schönborn

The main development was done by the [Graphics and Vision Research Group](http://gravis.cs.unibas.ch) at the University of Basel.

The team behind Statismo is now mainly focused on the Scala version of Statismo ([Scalismo](https://github.com/unibas-gravis/scalismo)).

Members of the [Laboratory Of Medical Information Processing](http://latim.univ-brest.fr/) decided to take over the C++ framework, though with limited resources.

Supported Languages
===================

The framework provides modules in modern c++ (c++17). Python wrappers
for the VTK module are also available.

Statismo vs Scalismo
====================

Apart from the core language (c++ vs Scala), Scalismo is designed around shape modeling whereas the main goal of Statismo is to add shape modeling
capabilities to other frameworks used in medical imaging (ITK and VTK for now).

Related Projects
================

* [Scalismo](http://github.com/unibas-gravis/scalismo) is a library for image analysis and shape modelling for the Java Virtual Machine. It is written in [Scala](www.scala-lang.org) and based on the same underlying concepts as statismo.

* [Deformetrica](http://www.deformetrica.org/) is a software, written in C++, for the statistical analysis of 2D and 3D shape data.

* [Spharm-PDM Toolbox](https://www.nitrc.org/projects/spharm-pdm) is a shape correspondence software package using a parametric boundary description based on spherical harmonics.

* [ITK](https://itk.org/) is an open-source, cross-platform library that provides developers with an extensive suite of software tools for image analysis.

* [VTK](https://vtk.org/) is an open source software for manipulating and displaying scientific data. It comes with state-of-the-art tools for 3D rendering, a suite of widgets for 3D interaction, and extensive 2D plotting capability.

* [HDF5](http://hdf5group.org) is an high performance data software library and file format to manage, process, and store your heterogeneous data.

Presentations
=====================

* [Statismo overview (Presentation slides)](https://github.com/downloads/statismo/statismo/statismo_teaser.pdf),
* [Statismo introduction, Presentation held at "Statismo - Get Together"  (Presentation slides)](https://docs.google.com/file/d/0BzIn1zFCNzg7cGxaRW9iUkhwWlE/edit?usp=sharing),
* [Unifying Shape model fitting and non-rigid registration (Presentation slides)](https://docs.google.com/file/d/0BzIn1zFCNzg7WnVvR1hmWFY1SEU/edit?usp=sharing),
* [Statismo tutorial, presentation held at Shape 2014](https://drive.google.com/file/d/0BzIn1zFCNzg7OGUtS1BrT1FDaE0/edit?usp=sharing),
* [A framework for PCA based statistical models ](https://edoc.unibas.ch/29542/).

Scientific Publications
=======================

* [Gaussian Process Morphable Models](https://ieeexplore.ieee.org/abstract/document/8010438),
* [Posterior shape models](https://www.sciencedirect.com/science/article/pii/S1361841513000844),
* [Spatially Varying Registration Using Gaussian Processes](https://link.springer.com/chapter/10.1007/978-3-319-10470-6_52).