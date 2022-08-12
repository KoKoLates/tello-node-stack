# Non-negative, Calibrated to take off, Auto-calibration if fall below take off height, inaccurate near ground
float32 height_m
float32 fight_time_s
float32 speed_easting_mps
float32 speed_northing_mps
float32 speed_vertival_mps
float32 speed_horizontal_mps

bool imu_state
bool wind_state
bool power_state
bool battery_state
bool gravity_state
bool pressure_state
bool down_visual_state

uint8 battery_percentage
uint8 imu_calibration_state
float32 drone_battery_left_s
float32 drone_fly_time_left_s

# Drone Status
bool on_flying
bool on_ground

# em_open True as flying, False as landing
bool em_open
bool drone_hover
bool battery_low
bool battery_lower
bool factory_mode
bool outage_recording

# fly_mode: 1 for landing, 6 for flying
uint8 fly_mode
uint8 camera_state
float32 throw_takeoff_timer_s
uint8 electrical_machinery_state

bool front_in
bool front_out
bool front_lsc
bool cmd_fast_mode

float32 cmd_yaw_ratio
float32 cmd_roll_ratio
float32 cmd_pitch_ratio
float32 cmd_vspeed_ratio
float32 temperature_height_m