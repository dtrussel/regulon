# =============================================================================
# package_smoke/CMakeLists.txt — Standalone consumer fixture proving that an
# installed Regulon package is usable via find_package(regulon).
#
# Not part of the regulon-c build tree: configured and built as its own
# project, pointed at an installed package via CMAKE_PREFIX_PATH, by the
# `package-install-smoke` CI job and (optionally) local package testing.
#
# RON-IS-001 §8.1
# SPDX-License-Identifier: MIT
# =============================================================================

cmake_minimum_required(VERSION 3.21)
project(regulon_package_smoke LANGUAGES C)

find_package(regulon REQUIRED)

add_executable(package_smoke main.c)
target_link_libraries(package_smoke PRIVATE regulon::regulon)
