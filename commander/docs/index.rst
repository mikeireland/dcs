.. commander documentation master file, created by
   sphinx-quickstart on Fri Mar  4 15:58:27 2022.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

.. only:: latex

Intro
=====

**commander - Command line generator and json parser for C++**

`commander` is a lightweight library that exposes C++ types in the command line
world and vice versa, mainly to create command line bindings of existing C++ code.
Its goals and syntax are similar to the excellent pybind11 library by Wenzel
Jakob: to minimize boilerplate code in traditional extension modules by inferring type
information using compile-time introspection.

.. only:: not latex

    Contents:

.. toctree::
   :caption: The Basics
   :maxdepth: 2

   installing
   basics
   functions
   classes

.. toctree::
   :caption: Extra Information
   :maxdepth: 1

   reference

