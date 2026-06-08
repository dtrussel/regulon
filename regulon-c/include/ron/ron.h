/*
 * @file     ron.h
 * @brief    Aggregate convenience header for the Regulon control-systems library.
 * @module   ron
 * @doc      RON-IS-001
 * @req      RON-DC-001, RON-QR-031
 * @version  1.0.0
 * SPDX-License-Identifier: MIT
 *
 * This optional umbrella header transitively includes every public Regulon
 * header so that consumers may pull in the whole library with a single
 *
 *     #include <ron/ron.h>
 *
 * Headers are included in dependency order, rooted at ron_platform.h, so the
 * include topology is exercised as an acyclic graph (RON-TC-INT-001).  Each
 * optional module header is guarded by its RON_HAVE_<MODULE> macro (from the
 * generated ron/ron_modules.h), so when the library is built with a subset of
 * modules this header includes only the headers whose implementations are
 * present.  The PID core and its integrated feed-forward path are the mandatory
 * baseline and are always included.
 *
 * Including individual ron/ron_*.h headers directly remains fully supported;
 * this header is purely a convenience and adds no symbols of its own.
 */

#ifndef RON_RON_H
#define RON_RON_H

/* Build-time module availability (generated). */
#include "ron/ron_modules.h"

/* ── Mandatory baseline ──────────────────────────────────────────────────── */
#include "ron/ron_feedforward.h"
#include "ron/ron_pid.h"
#include "ron/ron_pid_types.h"
#include "ron/ron_platform.h"

/* ── Optional PID-dependent modules ──────────────────────────────────────── */
#if RON_HAVE_GAIN_SCHED
#include "ron/ron_gain_sched.h"
#endif
#if RON_HAVE_CASCADE
#include "ron/ron_cascade.h"
#endif
#if RON_HAVE_AUTOTUNE
#include "ron/ron_autotune.h"
#endif

/* ── Optional standalone modules ─────────────────────────────────────────── */
#if RON_HAVE_FILTER
#include "ron/ron_filter.h"
#endif
#if RON_HAVE_TRAJECTORY
#include "ron/ron_trajectory.h"
#endif
#if RON_HAVE_HEALTH
#include "ron/ron_health.h"
#endif
#if RON_HAVE_METRICS
#include "ron/ron_metrics.h"
#endif

/* ── Optional matrix / state-estimation modules ──────────────────────────── */
#if RON_HAVE_KALMAN
#include "ron/ron_kalman.h"
#endif
#if RON_HAVE_STATESPACE
#include "ron/ron_observer.h"
#include "ron/ron_statespace.h"
#endif
#if RON_HAVE_LQR
#include "ron/ron_lqr.h"
#endif
#if RON_HAVE_LQG
#include "ron/ron_lqg.h"
#endif

#endif /* RON_RON_H */
