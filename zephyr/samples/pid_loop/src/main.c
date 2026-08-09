/*
 * @file     main.c
 * @brief    Regulon PID control loop running as a periodic Zephyr thread.
 * @doc      RON-IS-001
 * SPDX-License-Identifier: MIT
 *
 * Demonstrates the shape a real control loop takes under Zephyr: a dedicated
 * thread woken on a fixed period, a statically allocated controller instance,
 * and an explicit fault check on every step.
 *
 * The "plant" here is a first-order lag evaluated in software so the sample
 * runs anywhere, including native_sim. Replace read_plant()/drive_plant()
 * with a sensor read and an actuator write.
 *
 * Two details are worth copying rather than the arithmetic:
 *
 *   - The loop period is declared once and both the scheduler and the
 *     controller are driven from it, so dt can never silently disagree with
 *     the rate the loop actually runs at.
 *   - k_thread_deadline/k_timer are not used to fix up jitter. If your
 *     scheduler jitters meaningfully, measure the elapsed time and pass the
 *     measured value as dt instead of the nominal constant.
 */

#include <zephyr/kernel.h>
#include <ron/ron.h>

/* Loop rate. Both the sleep and the controller's dt derive from this. */
#define LOOP_PERIOD_MS 10
#define LOOP_DT_S      ((ron_float_t) LOOP_PERIOD_MS / 1000.0F)

#define LOOP_STACK_SIZE 2048
#define LOOP_PRIORITY   5

/* Controller state is caller-owned; the library never allocates. */
static ron_pid_instance_t pid;

/* Simulated first-order plant, stands in for sensor and actuator. */
static ron_float_t plant_state;

static ron_float_t read_plant(void)
{
    return plant_state;
}

static void drive_plant(ron_float_t u)
{
    /* y[k+1] = y[k] + dt * (u - y[k]) */
    plant_state += LOOP_DT_S * (u - plant_state);
}

static void control_loop(void *a, void *b, void *c)
{
    ron_pid_config_t cfg = {
        .Kp      = 2.0F,
        .Ki      = 5.0F,
        .Kd      = 0.0F,
        .b       = 1.0F,
        .c       = 1.0F,
        .u_min   = -10.0F,
        .u_max   = 10.0F,
        .I_min   = -100.0F,
        .I_max   = 100.0F,
        .aw_mode = RON_AW_BACK_CALC,
        .T_aw    = 0.05F,
    };
    const ron_float_t setpoint = 1.0F;
    k_timepoint_t next;

    ARG_UNUSED(a);
    ARG_UNUSED(b);
    ARG_UNUSED(c);

    if (ron_pid_init(&pid, &cfg) != RON_FAULT_NONE) {
        printk("regulon: PID configuration rejected\n");
        return;
    }

    for (int step = 0; step < 300; step++) {
        ron_float_t u;
        ron_status_t status;
        ron_fault_t fault;

        next = sys_timepoint_calc(K_MSEC(LOOP_PERIOD_MS));

        fault = ron_pid_step(&pid, setpoint, read_plant(), LOOP_DT_S, &u, &status);
        if (fault != RON_FAULT_NONE) {
            /* u still carries the configured safe-state output. Faults latch
             * until ron_pid_fault_clear(), so the loop will keep reporting
             * this until the cause is acknowledged.
             */
            printk("regulon: fault 0x%x at step %d\n", (unsigned int) fault, step);
            drive_plant(u);
            continue;
        }

        drive_plant(u);

        if ((step % 50) == 0) {
            printk("step %3d: y = %d milli, u = %d milli\n", step,
                   (int) (read_plant() * 1000.0F), (int) (u * 1000.0F));
        }

        k_sleep(sys_timepoint_timeout(next));
    }

    printk("regulon: final y = %d milli (setpoint %d milli)\n",
           (int) (read_plant() * 1000.0F), (int) (setpoint * 1000.0F));
}

K_THREAD_DEFINE(control_loop_tid, LOOP_STACK_SIZE, control_loop, NULL, NULL, NULL,
                LOOP_PRIORITY, 0, 0);

int main(void)
{
    printk("regulon PID loop sample, %d ms period\n", LOOP_PERIOD_MS);
    return 0;
}
