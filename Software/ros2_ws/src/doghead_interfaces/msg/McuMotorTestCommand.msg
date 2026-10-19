uint16 protocol_version
uint32 session_id
uint32 sequence
uint16 valid_for_ms

uint8 axis
# Joint/output-shaft RPM commanded for the selected axis (NOT motor shaft RPM).
# Despite the name "motor_rpm", this field carries joint-space RPM, consistent
# with Motor_SetSpeed() input and the bringup_motor_rpm_max cap in safety_validator.
# Gear ratio is applied exactly once, inside the MCU firmware driver.
# Maximum enforced value: bringup_motor_rpm_max (safety_validator.py).
float32 target_motor_rpm
float32 requested_accel_rpm_s   # Joint/output-shaft RPM/s
float32 requested_jerk_rpm_s2   # Joint/output-shaft RPM/s^2
bool enable
