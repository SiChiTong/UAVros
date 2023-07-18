Header header

#mode 0:home 1:tracking 2：yaw follo
uint8 mode

#Current gimbal imu roll, pitch, yaw angles, published at 30 Hz.
float32[3] imu_angle

#Current gimbal joint angles, published at 30 Hz.
float32[3] rotor_angle