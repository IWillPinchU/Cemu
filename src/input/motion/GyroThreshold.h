#pragma once

#include <cmath>

namespace Motion
{
inline constexpr float kDefaultGyroThresholdTurnsPerSecond = 0.007f;
inline constexpr float kMaximumGyroThresholdTurnsPerSecond = 0.1f;
inline constexpr float kTurnsPerSecondToRadiansPerSecond = 2.0f * 3.14159265f;

inline bool IsGyroRateBelowThreshold(float gxRadiansPerSecond, float gyRadiansPerSecond,
                                    float gzRadiansPerSecond, float thresholdTurnsPerSecond)
{
	const float thresholdRadiansPerSecond = thresholdTurnsPerSecond * kTurnsPerSecondToRadiansPerSecond;
	return std::sqrt(gxRadiansPerSecond * gxRadiansPerSecond + gyRadiansPerSecond * gyRadiansPerSecond +
	                 gzRadiansPerSecond * gzRadiansPerSecond) < thresholdRadiansPerSecond;
}
} // namespace Motion
