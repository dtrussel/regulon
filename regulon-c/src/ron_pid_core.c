/*
 * @file     ron_pid_core.c
 * @brief    PID controller core computation pipeline.
 * @module   ron_pid_core
 * @doc      RON-IS-001
 * @req      RON-FR-001, RON-FR-002, RON-FR-003, RON-FR-004, RON-FR-005,
 *           RON-FR-006, RON-FR-007, RON-FR-010, RON-FR-011, RON-FR-012,
 *           RON-FR-013, RON-FR-020, RON-FR-021, RON-FR-022, RON-FR-025,
 *           RON-FR-026, RON-FR-027, RON-FR-030, RON-FR-031, RON-FR-032,
 *           RON-FR-033, RON-FR-034, RON-FR-035, RON-FR-054, RON-FR-070,
 *           RON-SR-010, RON-SR-011, RON-SR-013, RON-SR-020
 * @version  1.0.0
 * @author   TBD
 * SPDX-License-Identifier: MIT
 *
 * Implements the 17-stage computation pipeline specified in
 * SADS_ControlLib.rst §"Module Design: ron_pid_core".  All sub-operations
 * are inlined (static inline helpers) into a single bounded function to
 * minimise call overhead and simplify WCET analysis (SADS §552).
 *
 * Sprint status: Sprint 2 — full implementation.
 */

#include "ron/ron_pid.h"
#include "ron_pid_internal.h"

/* =========================================================================
 * Sub-operation: compute_normalise
 *
 * Linear map from engineering units to the dimensionless domain.
 * Division safety is guaranteed by ron_pid_config_validate() which
 * rejects configurations with in_max <= in_min when normalise=true.
 *
 * Satisfies: RON-FR-010, RON-FR-011 | Test: RON-TC-PID-011, RON-TC-PID-012
 * ========================================================================= */
static inline ron_float_t core_compute_normalise(ron_float_t x,
                                                 ron_float_t x_min,
                                                 ron_float_t x_max)
{
    return (x - x_min) / (x_max - x_min);
}

/* =========================================================================
 * Sub-operation: compute_denormalise
 *
 * Inverse of compute_normalise — maps back to engineering units.
 *
 * Satisfies: RON-FR-012 | Test: RON-TC-PID-013
 * ========================================================================= */
static inline ron_float_t core_compute_denormalise(ron_float_t x_norm,
                                                   ron_float_t out_min,
                                                   ron_float_t out_max)
{
    return (x_norm * (out_max - out_min)) + out_min;
}

/* =========================================================================
 * Sub-operation: compute_sp_filter
 *
 * First-order IIR setpoint filter, Euler-discretised:
 *   r_f(k) = α · r_f(k-1) + (1 - α) · r(k),   α = τ / (τ + dt).
 * Returns r unchanged when tau_sp <= 0 (filter disabled).
 *
 * Satisfies: RON-FR-053 (runtime update), SADS §compute_sp_filter.
 * ========================================================================= */
static inline ron_float_t core_compute_sp_filter(ron_float_t r,
                                                 ron_float_t r_f_prev,
                                                 ron_float_t tau_sp,
                                                 ron_float_t dt)
{
    ron_float_t out;
    if (tau_sp <= RON_FLOAT_C(0.0))
    {
        out = r;
    }
    else
    {
        ron_float_t alpha = tau_sp / (tau_sp + dt);
        out = (alpha * r_f_prev) + ((RON_FLOAT_C(1.0) - alpha) * r);
    }
    return out;
}

/* =========================================================================
 * Sub-operation: compute_derivative
 *
 * Computes D(k) using either derivative-on-error or derivative-on-
 * measurement, with an optional Tustin-discretised first-order low-pass
 * filter when cfg.N > 0:
 *
 *   D_f(k) = (N·dt / (1 + N·dt)) · D_raw
 *          + (    1 / (1 + N·dt)) · D_f(k-1)
 *
 * RON-FR-005 mandates DoM as the default; RON-FR-006 mandates the
 * filter.  When N == 0 the filter is disabled.
 *
 * Satisfies: RON-FR-005, RON-FR-006 | Test: RON-TC-PID-006 – RON-TC-PID-009
 * ========================================================================= */
static inline ron_float_t core_compute_derivative(const ron_pid_config_t *cfg,
                                                  const ron_pid_state_t  *state,
                                                  ron_float_t             y_n,
                                                  ron_float_t             e_D,
                                                  ron_float_t             dt)
{
    /* Select raw derivative delta per mode. */
    ron_float_t delta;
    if (cfg->deriv_mode == RON_DERIV_ON_MEASUREMENT)
    {
        /* D = -Kd · (y - y_prev)/dt  → sign flipped via delta. */
        delta = -(y_n - state->y_prev);
    }
    else
    {
        delta = e_D - state->e_D_prev;
    }

    ron_float_t D_raw = (cfg->Kd * delta) / dt;

    /* Apply Tustin LP filter when N > 0, else pass-through. */
    ron_float_t D_f;
    if (cfg->N > RON_FLOAT_C(0.0))
    {
        ron_float_t Ndt  = cfg->N * dt;
        ron_float_t denom = RON_FLOAT_C(1.0) + Ndt;
        D_f = ((Ndt / denom) * D_raw) +
              ((RON_FLOAT_C(1.0) / denom) * state->D_filt_prev);
    }
    else
    {
        D_f = D_raw;
    }
    return D_f;
}

/* =========================================================================
 * Sub-operation: compute_integral
 *
 * Computes the integral update with anti-windup:
 *   - Euler       : I_update = Ki · dt · e(k)
 *   - Trapezoidal : I_update = Ki · dt · 0.5 · (e(k) + e(k-1))
 *
 * Anti-windup options:
 *   - RON_AW_BACK_CALC : I_new += (dt/T_aw) · (u_sat_prev − u_prev)
 *   - RON_AW_CLAMPING  : suppress I_update while (saturated_last_step
 *                        AND sign(e) == sign(I))
 *   - RON_AW_NONE      : plain integration
 * The final I_new is hard-clamped to [I_min, I_max].
 *
 * Satisfies: RON-FR-004, RON-FR-030..FR-035 | Test: RON-TC-PID-004,
 *            RON-TC-PID-005, RON-TC-PID-021 – RON-TC-PID-026
 * ========================================================================= */
static inline ron_float_t core_compute_integral(const ron_pid_config_t *cfg,
                                                const ron_pid_state_t  *state,
                                                ron_float_t             e,
                                                ron_float_t             dt)
{
    /* Rectangular (Euler) or trapezoidal (Tustin) update. */
    ron_float_t I_update;
    if (cfg->integ_method == RON_INTEG_TRAPEZOIDAL)
    {
        I_update = (cfg->Ki * dt) *
                   (RON_FLOAT_C(0.5) * (e + state->e_prev));
    }
    else
    {
        I_update = cfg->Ki * dt * e;
    }

    /* Back-calculation anti-windup correction term. */
    ron_float_t aw_correction = RON_FLOAT_C(0.0);
    if (cfg->aw_mode == RON_AW_BACK_CALC)
    {
        aw_correction = (dt / cfg->T_aw) *
                        (state->u_sat_prev - state->u_prev);
    }

    ron_float_t I_new = state->integral + I_update + aw_correction;

    /* Conditional integration clamping — suppress if winding deeper. */
    if (cfg->aw_mode == RON_AW_CLAMPING)
    {
        bool saturated = (state->u_sat_prev <= cfg->u_min) ||
                         (state->u_sat_prev >= cfg->u_max);
        bool same_sign = ((e > RON_FLOAT_C(0.0)) &&
                          (state->integral > RON_FLOAT_C(0.0))) ||
                         ((e < RON_FLOAT_C(0.0)) &&
                          (state->integral < RON_FLOAT_C(0.0)));
        if (saturated && same_sign)
        {
            I_new = state->integral;
        }
    }

    /* Hard integral clamp (RON-FR-035). */
    I_new = ron_clamp(I_new, cfg->I_min, cfg->I_max);
    return I_new;
}

/* =========================================================================
 * ron_pid_core_step — primary runtime pipeline.
 * ========================================================================= */

/* Satisfies: RON-FR-001 – RON-FR-035, RON-FR-054, RON-FR-070 |
 * Test: RON-TC-PID-001 – RON-TC-PID-026                             */
ron_fault_t ron_pid_core_step(ron_pid_instance_t *inst,
                              ron_float_t         r,
                              ron_float_t         y,
                              ron_float_t         dt,
                              ron_float_t        *u_out,
                              ron_status_t       *status)
{
    ron_pid_config_t *cfg   = &inst->config;
    ron_pid_state_t  *state = &inst->state;

    /* ── 1. Input NaN/Inf guard ───────────────────────────────────── */
    if (!RON_ISFINITE(r) || !RON_ISFINITE(y))
    {
        ron_pid_fault_set(inst, RON_FAULT_INPUT_NAN);
        *u_out  = ron_pid_fault_safe_output(inst);
        *status = state->status;
        return RON_FAULT_INPUT_NAN;
    }

    /* ── 2. Manual-mode passthrough ───────────────────────────────── */
    if (state->mode == RON_MODE_MANUAL)
    {
        ron_status_t st = (ron_status_t)RON_STATUS_MANUAL_MODE;
        state->status   = st;
        *u_out  = state->u_sat_prev;
        *status = st;
        return RON_FAULT_NONE;
    }

    /* ── 3. Input normalisation ───────────────────────────────────── */
    ron_float_t r_n;
    ron_float_t y_n;
    if (cfg->normalise)
    {
        r_n = core_compute_normalise(r, cfg->in_min, cfg->in_max);
        y_n = core_compute_normalise(y, cfg->in_min, cfg->in_max);
    }
    else
    {
        r_n = r;
        y_n = y;
    }

    /* ── 4. Setpoint filter ───────────────────────────────────────── */
    ron_float_t r_f = core_compute_sp_filter(r_n, state->r_filt_prev,
                                             cfg->tau_sp, dt);

    /* ── 5. Setpoint-change integral reset (RON-FR-054) ───────────── */
    if (cfg->sp_reset_threshold > RON_FLOAT_C(0.0))
    {
        if (ron_fabs(r_f - state->r_filt_prev) > cfg->sp_reset_threshold)
        {
            state->integral = RON_FLOAT_C(0.0);
        }
    }

    /* ── 6. Error signals (2DOF, RON-FR-007) ──────────────────────── */
    ron_float_t e    = r_f - y_n;
    ron_float_t e_P  = (cfg->b * r_f) - y_n;
    ron_float_t e_D  = (cfg->c * r_f) - y_n;

    /* ── 7. Proportional term ─────────────────────────────────────── */
    ron_float_t P = cfg->Kp * e_P;

    /* ── 8. Derivative term (with optional LP filter) ─────────────── */
    ron_float_t D_f = core_compute_derivative(cfg, state, y_n, e_D, dt);

    /* ── 9. Integral term (with anti-windup) ──────────────────────── */
    ron_float_t I_new = core_compute_integral(cfg, state, e, dt);

    /* ── 10. Raw output ───────────────────────────────────────────── */
    ron_float_t u_raw = P + I_new + D_f;

    /* ── 11. Denormalise output ───────────────────────────────────── */
    ron_float_t u_eng;
    if (cfg->normalise)
    {
        u_eng = core_compute_denormalise(u_raw, cfg->out_min, cfg->out_max);
    }
    else
    {
        u_eng = u_raw;
    }

    /* ── 12. Output NaN/Inf guard ─────────────────────────────────── */
    if (!RON_ISFINITE(u_eng))
    {
        ron_pid_fault_set(inst, RON_FAULT_OUTPUT_NAN);
        *u_out  = ron_pid_fault_safe_output(inst);
        *status = state->status;
        return RON_FAULT_OUTPUT_NAN;
    }

    /* Status word construction begins here. */
    ron_status_t st = RON_STATUS_OK;
    if (cfg->normalise)
    {
        st = (ron_status_t)(st | (ron_status_t)RON_STATUS_NORMALISED);
    }
    if (cfg->tau_sp > RON_FLOAT_C(0.0))
    {
        st = (ron_status_t)(st | (ron_status_t)RON_STATUS_SP_FILTER_ACTIVE);
    }

    /* ── 13. Output saturation ────────────────────────────────────── */
    ron_float_t u_sat = ron_clamp(u_eng, cfg->u_min, cfg->u_max);
    bool saturated = (u_sat != u_eng);
    if (saturated)
    {
        st = (ron_status_t)(st | (ron_status_t)RON_STATUS_SATURATED);
    }

    /* ── 14. Rate limiting ────────────────────────────────────────── */
    ron_float_t u_final = u_sat;
    if (cfg->du_max > RON_FLOAT_C(0.0))
    {
        ron_float_t delta_max = cfg->du_max * dt;
        ron_float_t delta     = u_sat - state->u_sat_prev;
        if (delta > delta_max)
        {
            u_final = state->u_sat_prev + delta_max;
            st = (ron_status_t)(st | (ron_status_t)RON_STATUS_RATE_LIMITED);
        }
        else if (delta < -delta_max)
        {
            u_final = state->u_sat_prev - delta_max;
            st = (ron_status_t)(st | (ron_status_t)RON_STATUS_RATE_LIMITED);
        }
        else
        {
            /* within rate budget — no change */
        }
    }

    /* ── 15. Integral overflow check ──────────────────────────────── */
    if (cfg->I_overflow_thresh > RON_FLOAT_C(0.0))
    {
        if (ron_fabs(I_new) > cfg->I_overflow_thresh)
        {
            ron_pid_fault_set(inst, RON_FAULT_INTEGRAL_OVERFLOW);
            *u_out  = ron_pid_fault_safe_output(inst);
            *status = state->status;
            return RON_FAULT_INTEGRAL_OVERFLOW;
        }
    }

    /* ── 17. AW-active flag ───────────────────────────────────────── */
    if ((cfg->aw_mode != RON_AW_NONE) && saturated)
    {
        st = (ron_status_t)(st | (ron_status_t)RON_STATUS_AW_ACTIVE);
    }

    /* ── 16. State writeback ──────────────────────────────────────── */
    state->integral    = I_new;
    state->y_prev      = y_n;
    state->e_D_prev    = e_D;
    state->D_filt_prev = D_f;
    state->r_filt_prev = r_f;
    state->u_sat_prev  = u_final;
    state->u_prev      = u_eng;   /* pre-saturation for AW back-calc */
    state->e_prev      = e;
    state->status      = st;

    *u_out  = u_final;
    *status = st;
    return RON_FAULT_NONE;
}
