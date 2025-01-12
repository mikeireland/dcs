.. _installing:

Installing the library
######################

There are several ways to get the commander source, which lives at
`raplonu/commander on GitHub <https://github.com/raplonu/commander>`_. The commander
developers recommend one of the two ways listed here for obtaining commander.

.. _conan_remote_package:

Include with conan remote
=========================

You can obtain commander from the cosmic conan remote. For that you need to
add the following url to your conan remote list. To do that, type the following
command in your terminal:

.. code-block:: bash

    conan remote add cosmic https://odp2.jfrog.io/artifactory/api/conan/cosmic

Then adds the commander dependency to your conanfile.txt file.

.. code-block:: text

    [requires]
    commander/0.2.0

.. seealso::

    For more information about conan, `conan.io <https://conan.io>`_.

.. seealso::

    For a complete example using conan take a look at the example directory.

Include with conan local
========================

It is also possible to locally create the commander package from source.
For that, you need to clone the project locally and then ask conan to create the
commander package.

.. code-block:: bash

    git clone https://github.com/raplonu/commander.git
    conan create commander

This will create a new package in your local conan cache.
You can find it by typing the following command in your terminal:

.. code-block:: bash

    conan search commander/0.2.0

Finally, you can use the package in your project the same way as if you had used
the package from the cosmic conan remote.