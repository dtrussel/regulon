/*
 * @file     math.h
 * @brief    Minimal ARMv7 freestanding math declarations for Clang smoke builds.
 * @doc      RON-IS-001
 * @req      RON-DC-005
 * @version  1.0.0
 * SPDX-License-Identifier: MIT
 */

#ifndef RON_FREESTANDING_ARMV7_MATH_H
#define RON_FREESTANDING_ARMV7_MATH_H

#ifdef __cplusplus
extern "C" {
#endif

double cos(double x);
double sin(double x);

#ifdef __cplusplus
}
#endif

#endif /* RON_FREESTANDING_ARMV7_MATH_H */
