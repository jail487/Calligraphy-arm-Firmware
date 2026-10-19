#define HOST_TEST 1

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "nidec_motor_driver.h"

int main(void) {
    Motor_t motor;
    memset(&motor, 0, sizeof(motor));
    motor.last_counter_val = 12U;
    motor.total_pulse_count = 0x100000012LL;

    if (Motor_GetTotalPulseCount(&motor) != 0x100000012LL) {
        printf("FAIL: cumulative encoder count accessor\n");
        return 1;
    }
    if (Motor_GetTotalPulseCount(&motor) == (int64_t)motor.last_counter_val) {
        printf("FAIL: telemetry must not use timer snapshot\n");
        return 1;
    }
    printf("PASS: cumulative encoder count accessor\n");
    return 0;
}
