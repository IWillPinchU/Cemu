#pragma once

namespace DSUMotion
{
inline constexpr float kGyroRateScaleRadiansPerSecond = 2.0f * 3.14159265f / 312.0f;

// DSU data order is pitch, yaw, roll; Eden maps it to pitch, roll, negative yaw.
inline void MapGyro(float pitch, float yaw, float roll, float gyro[3])
{
	gyro[0] = pitch * kGyroRateScaleRadiansPerSecond;
	gyro[1] = roll * kGyroRateScaleRadiansPerSecond;
	gyro[2] = -yaw * kGyroRateScaleRadiansPerSecond;
}

inline void MapAcceleration(float x, float y, float z, float acceleration[3])
{
	acceleration[0] = x;
	acceleration[1] = -z;
	acceleration[2] = y;
}
} // namespace DSUMotion
