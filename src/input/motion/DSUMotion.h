#pragma once

namespace DSUMotion
{
inline constexpr float kGyroRateScaleRadiansPerSecond = 2.0f * 3.14159265f / 360.0f;

// DSU gyro fields are interpreted as degrees/second: raw * 2pi/360 gives radians/second.
// Cemu's native filter coordinate system expects: gx = pitch, gy = yaw, gz = roll.
inline void MapGyro(float pitch, float yaw, float roll, float gyro[3])
{
	gyro[0] = pitch * kGyroRateScaleRadiansPerSecond;
	gyro[1] = yaw * kGyroRateScaleRadiansPerSecond;
	gyro[2] = roll * kGyroRateScaleRadiansPerSecond;
}

// Cemu's native accelerometer mapping: ax = x, ay = -y, az = -z.
inline void MapAcceleration(float x, float y, float z, float acceleration[3])
{
	acceleration[0] = x;
	acceleration[1] = -y;
	acceleration[2] = -z;
}
} // namespace DSUMotion
