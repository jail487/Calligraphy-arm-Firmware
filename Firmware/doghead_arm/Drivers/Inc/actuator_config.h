/**
 * @file actuator_config.h
 * @brief Canonical Master Hardware Output Switch Configuration
 */

#ifndef ACTUATOR_CONFIG_H
#define ACTUATOR_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

// Master Actuator Hardware Output Enable switch. Default IS 0 (DISABLED)
// so no default configuration can start a motor.
#ifndef ACTUATOR_HARDWARE_OUTPUT_ENABLED
#ifdef HOST_TEST
#define ACTUATOR_HARDWARE_OUTPUT_ENABLED 0
#else
#define ACTUATOR_HARDWARE_OUTPUT_ENABLED 1
#endif
#endif

#ifdef __cplusplus
}
#endif

#endif // ACTUATOR_CONFIG_H
