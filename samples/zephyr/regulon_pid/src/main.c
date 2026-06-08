/*
 * @file     main.c
 * @brief    Zephyr sample: drive a first-order plant to setpoint with a Regulon PID.
 * @doc      RON-IS-001
 * @req      RON-FR-001
 * @version  1.0.0
 * SPDX-License-Identifier: MIT
 *
 * Demonstrates consuming the Regulon library as a Zephyr module. Builds against
 * Zephyr's minimal libc (no libm). Prints a single PASS/FAIL line that the
 * twister console harness matches (RON-TC-QUAL-023).
 */

#include <zephyr/kernel.h>

#include <ron/ron.h>

int main(void)
{
	ron_pid_instance_t pid;
	ron_pid_config_t cfg = {0};
	const ron_float_t dt = RON_FLOAT_C(0.01);
	ron_float_t y        = RON_FLOAT_C(0.0); /* plant output */
	unsigned int k;

	cfg.Kp          = RON_FLOAT_C(2.0);
	cfg.Ki          = RON_FLOAT_C(5.0);
	cfg.b           = RON_FLOAT_C(1.0);
	cfg.c           = RON_FLOAT_C(1.0);
	cfg.u_min       = RON_FLOAT_C(-10.0);
	cfg.u_max       = RON_FLOAT_C(10.0);
	cfg.I_min       = RON_FLOAT_C(-100.0);
	cfg.I_max       = RON_FLOAT_C(100.0);
	cfg.aw_mode     = RON_AW_BACK_CALC;
	cfg.T_aw        = RON_FLOAT_C(0.05);
	cfg.safe_policy = RON_SAFE_HOLD_LAST;

	if (ron_pid_init(&pid, &cfg) != RON_FAULT_NONE) {
		printk("Regulon PID sample: FAIL (init)\n");
		return 0;
	}

	for (k = 0U; k < 400U; ++k) {
		const ron_float_t tau = RON_FLOAT_C(0.2);
		ron_float_t u         = RON_FLOAT_C(0.0);
		ron_status_t status;

		(void)ron_pid_step(&pid, RON_FLOAT_C(1.0), y, dt, &u, &status);
		y = y + (dt / tau) * (u - y); /* first-order plant */
	}

	/* Converged to within 2% of the unit setpoint? */
	if (ron_fabs(y - RON_FLOAT_C(1.0)) < RON_FLOAT_C(0.02)) {
		printk("Regulon PID sample: PASS\n");
	} else {
		printk("Regulon PID sample: FAIL (no convergence)\n");
	}
	return 0;
}
