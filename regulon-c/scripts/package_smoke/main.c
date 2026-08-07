/*
 * @file     main.c
 * @brief    Package-install smoke test: link and run against an installed
 *           Regulon package resolved via find_package(regulon).
 * @doc      RON-IS-001
 * @version  1.0.0
 * SPDX-License-Identifier: MIT
 */

#include <stdio.h>

#include "ron/ron.h"

int main(void)
{
    ron_pid_instance_t pid;
    ron_pid_config_t cfg = {0};
    ron_fault_t fault;

    cfg.Kp    = RON_FLOAT_C(1.0);
    cfg.u_min = RON_FLOAT_C(-1.0);
    cfg.u_max = RON_FLOAT_C(1.0);
    cfg.I_min = RON_FLOAT_C(-1.0);
    cfg.I_max = RON_FLOAT_C(1.0);

    fault = ron_pid_init(&pid, &cfg);
    (void) printf("package_smoke: ron_pid_init -> %d\n", (int) fault);
    return (fault == RON_FAULT_NONE) ? 0 : 1;
}
