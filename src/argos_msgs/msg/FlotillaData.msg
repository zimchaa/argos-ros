std_msgs/Header header
# Weather module (BMP280)
float32 temperature_c
float32 pressure_hpa
# Body Motion module (LSM303D ch6) — accelerometer in g, mag in raw counts
float32 body_acc_x
float32 body_acc_y
float32 body_acc_z
int32   body_mag_x
int32   body_mag_y
int32   body_mag_z
float32 body_heading    # tilt-compensated heading degrees 0-360
# Arm Motion module (LSM303D ch1) — accelerometer in g, mag in raw counts
float32 arm_acc_x
float32 arm_acc_y
float32 arm_acc_z
int32   arm_mag_x
int32   arm_mag_y
int32   arm_mag_z
# Validity flags
bool has_weather
bool has_body_motion
bool has_arm_motion
