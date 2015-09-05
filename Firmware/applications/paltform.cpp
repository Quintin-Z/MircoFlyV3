#include <sensor.h>
#include <mpu9250.hpp>

MPU9250_Accelerometer acc = MPU9250_Accelerometer("mpu6500");
MPU9250_Gyroscope gryo = MPU9250_Gyroscope("mpu6500");
MPU9250_Magnetometer mag = MPU9250_Magnetometer("mpu6500");
