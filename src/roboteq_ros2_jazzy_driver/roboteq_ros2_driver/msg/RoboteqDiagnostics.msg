# Added separately so existing RoboteqStatus bags and subscribers stay compatible.
std_msgs/Header header
uint32 STOP_SOFTWARE=1
uint32 STOP_LOW_VOLTAGE=2
uint32 STOP_MD_FAULT=4
uint32 STOP_TELEMETRY_TIMEOUT=8
uint32 STOP_STARTUP=16
uint32 STOP_RELEASE_REJECTED=32

bool closed_loop
bool control_sample_valid
builtin_interfaces/Time control_stamp
float64 control_dt
# Speeds used by the ROS FF+PI controller, in m/s (feedback is filtered).
float64 target_speed_right
float64 target_speed_left
float64 feedback_speed_right
float64 feedback_speed_left
float64 error_right
float64 error_left
# Integral state: m/s*s. FF/P/I/output terms: !G command units.
float64 integral_right
float64 integral_left
float64 feedforward_right
float64 feedforward_left
float64 p_term_right
float64 p_term_left
float64 i_term_right
float64 i_term_left
float64 output_unlimited_right
float64 output_unlimited_left
float64 output_limited_right
float64 output_limited_left

# Successfully written serial commands; not proof the MD applied the command.
int32 sent_command_ch1
int32 sent_command_ch2
bool command_write_ok_ch1
bool command_write_ok_ch2
float64 command_age_ch1
float64 command_age_ch2

# *_valid means a complete response was received within the last 0.5 seconds.
# Invalid raw values must never be interpreted as healthy zero.
int32 status_flags
bool status_flags_valid
float64 status_flags_age
string[] status_flag_labels
int32 motor_flags_ch1
int32 motor_flags_ch2
bool motor_flags_valid_ch1
bool motor_flags_valid_ch2
float64 motor_flags_age_ch1
float64 motor_flags_age_ch2
string[] motor_flag_labels_ch1
string[] motor_flag_labels_ch2
# Raw ?P response; controller motor-output units, NOT watts.
float64 md_output_ch1
float64 md_output_ch2
bool md_output_valid_ch1
bool md_output_valid_ch2
float64 md_output_age_ch1
float64 md_output_age_ch2

bool software_estop
bool hardware_estop
bool estop_active
uint32 latched_stop_reason_mask
string[] latched_stop_reasons
# Most recent stop event, captured before issuing the MD emergency-stop command.
builtin_interfaces/Time last_stop_stamp
uint32 last_stop_reason_mask
string[] last_stop_reasons
bool last_stop_telemetry_valid
float64 last_stop_voltage
int32 last_stop_fault_flags
float64 voltage_trip_threshold
float64 voltage_recovery_threshold
