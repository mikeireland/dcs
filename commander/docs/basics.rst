.. _basics:

First steps
###########

This sections demonstrates the basic features of commander. Before getting
started, make sure that development environment is set up.

Compiling
=========

Linux/macOS
-----------

OYou'll need to install the **conan** packages as well as **cmake**. Both can be
found in **pypi** using pip:

.. code-block:: bash

    pip install conan cmake

Conan need to be configured with a profile and the cosmic remote. A script is
provided: **tools/configure_conan.sh**.

After installing the prerequisites, run

.. code-block:: bash

   ./configure.sh
   ./build.sh

Header and namespace conventions
================================

For brevity, all code examples assume that the following two lines are present:

.. code-block:: cpp

    #include <commander/commander.h>

    namespace co = commander;

.. _simple_example:

Creating bindings for a simple function
=======================================

Let's start by creating command bindings for an extremely simple function, which
adds two numbers and returns their result:

.. code-block:: cpp

    int add(int i, int j) {
        return i + j;
    }

For simplicity [#f1]_, we'll put both this function and the binding code into
a file named :file:`example.cpp` with the following contents:

.. code-block:: cpp

    #include <commander/commander.h>

    int add(int i, int j) {
        return i + j;
    }

    COMMANDER_REGISTER(m) {
        m.def("add", &add, "A function that adds two numbers");
    }

.. [#f1] In practice, implementation and binding code will generally be located
         in separate files.

The :func:`COMMANDER_REGISTER` macro creates a function that will be called when an
:struct:`commander::Module` instance is issued in the main. The argument (``m``) defines a variable
of type :struct:`commander::Module` which
is the main interface for creating bindings. The method :func:`commander::Module::def`
generates binding code that exposes the ``add()`` function to Python.

.. note::

    Notice how little code was needed to expose our function to Python: all
    details regarding the function's parameters and return value were
    automatically inferred using template metaprogramming. This overall
    approach and the used syntax are borrowed from Boost.Python, though the
    underlying implementation is very different.
