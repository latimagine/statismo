Data Description
================

This data contains shapes (polydata), images and deformation fields of
2D hand shapes. All data is in correspondence and can be used directly to build simple shape,
image and deformation models.

File Hierarchy
==============

~~~
├── femur_meshes    : femur bone meshes
├── hand_dfs        : hand deformation fields
├── hand_images     : hand images with surrogate data
├── hand_landmarks  : hand landmarks
├── hand_polydata   : hand meshes
~~~

Surrogate Format
================

The surrogate system used in conditional model building relies
on the following descriptions files:

* ***xxx_surrogates_types.txt***

Description of the surrogate type: continuous (type 1) or categorical (type 0) data.

Example for a categorical variable (gender) and a continuous one (height):
~~~
0
1
~~~
* ***xxx_surrogates.txt***

Surrogate values bound to the dataset.

Example for a categorical variable (gender, 0=male, 1=female) and a continuous one (height in kg):
~~~
0
54.3
~~~
* ***conditioning_information.txt***

Description of the variables a conditional model should
be built against. First column indicates whether the variable is used for conditioning (0=no, 1=yes) and the second indicates values for the condition (the value is ignored if the variable is not used for conditioning)

Example for a model build against height only:
~~~
0 1
1 48.3
~~~


Usage
=====

* See VTK/ITK modules example folder
* See ITM module cli tools test script