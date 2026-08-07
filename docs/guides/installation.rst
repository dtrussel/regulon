Installation and Integration
============================

There are three ways to consume Regulon. Pick by how you manage
dependencies, not by project size — all three build the same library.

Install, then find_package
--------------------------

Best when the library is shared between projects or packaged by a
distribution.

.. code-block:: bash

   cmake -B build -S regulon-c -DCMAKE_INSTALL_PREFIX=/opt/regulon
   cmake --build build
   cmake --install build

This installs the static library, the public headers, a relocatable CMake
package, and a pkg-config file. Consumers then need only:

.. code-block:: cmake

   find_package(regulon REQUIRED)
   target_link_libraries(my_target PRIVATE regulon::regulon)

If the prefix is not a default search location, point CMake at it with
``-DCMAKE_PREFIX_PATH=/opt/regulon``.

The package declares ``SameMajorVersion`` compatibility, so
``find_package(regulon 0.1 REQUIRED)`` accepts any ``0.x`` release but
refuses ``1.0``.

Vendor the source tree
----------------------

Best when you want the library pinned to the exact revision you tested, with
no install step and no external state:

.. code-block:: cmake

   add_subdirectory(third_party/regulon/regulon-c)
   target_link_libraries(my_target PRIVATE regulon::regulon)

The ``regulon::regulon`` alias exists in both the in-tree and installed
cases, so code written for one works unchanged with the other.

Build options are ordinary CMake cache variables. When vendoring, set them
before ``add_subdirectory``:

.. code-block:: cmake

   set(RON_BUILD_TESTS OFF CACHE BOOL "")
   set(RON_USE_DOUBLE  ON  CACHE BOOL "")
   add_subdirectory(third_party/regulon/regulon-c)

Non-CMake builds
----------------

A pkg-config file is installed alongside the library:

.. code-block:: bash

   gcc app.c $(pkg-config --cflags --libs regulon) -o app

For a hand-written Makefile, the library is a single static archive with no
dependencies beyond the C standard library — add the include directory and
link ``libregulon.a`` directly.

Choosing precision
------------------

Regulon computes in ``ron_float_t``, which is ``float`` by default.
``-DRON_USE_DOUBLE=ON`` makes it ``double``.

This is a **whole-library** decision, not a per-module one, and it changes
the ABI. The library and everything that includes its headers must agree.
When consuming an installed package this is handled for you: the option is
recorded as a ``PUBLIC`` compile definition on the exported target, so
consumers inherit it automatically.

Single precision is the right default for embedded targets with a
single-precision FPU, where ``double`` silently falls back to slow software
emulation. Prefer ``double`` when the plant model is stiff, when state
covariances span many orders of magnitude, or when a matrix-based module
struggles to converge — the :doc:`../api/lqr` DARE solver is the most
sensitive to this.

Verifying the install
---------------------

The repository includes a consumer fixture used by CI to prove the exported
package actually resolves. It is a useful sanity check after installing to a
new prefix:

.. code-block:: bash

   cmake -B /tmp/smoke -S regulon-c/scripts/package_smoke \
         -DCMAKE_PREFIX_PATH=/opt/regulon
   cmake --build /tmp/smoke
   /tmp/smoke/package_smoke
