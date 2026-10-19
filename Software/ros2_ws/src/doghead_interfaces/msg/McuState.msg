uint16 protocol_version
uint32 boot_id
uint32 state_sequence
uint32 uptime_ms

uint8 control_mode
uint8 safety_state
uint16 fault_code
# Incoming command enable mask requested by host.
uint8 requested_enable_mask
# Validated/logically applied enable mask.  It is not proof that physical
# actuator output is enabled; the final MCU hardware/safety gate is separate.
uint8 enable_mask
# Actuator enable mask that passed the final physical safety gate.
uint8 physical_enable_mask
bool agent_connected

float32[3] target_position_rad
float32[3] target_velocity_rad_s
float32[3] target_acceleration_rad_s2

float32[3] applied_position_rad
float32[3] applied_velocity_rad_s

float32[3] measured_position_rad
float32[3] measured_velocity_rad_s
float32[3] position_error_rad

# Joint/output-shaft RPM commanded by the MCU controller after validation.
# Despite the name "motor_rpm", this is NOT motor shaft RPM.
# Gear ratio is applied exactly once inside the firmware motor driver.
# Distinct from measured_velocity_rad_s which comes from encoder feedback.
float32[3] commanded_motor_rpm
int64[3] encoder_count
uint32[3] pwm_frequency_hz

uint16 command_age_ms
uint32 last_accepted_command_sequence
uint32 rejected_command_total
