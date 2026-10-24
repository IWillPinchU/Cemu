#define NOMINMAX
#include <cctype>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <string>
#include <string_view>

using sint32 = int32_t;
using uint32 = uint32_t;
using uint64 = uint64_t;
using sint64 = int64_t;
using uint8 = uint8_t;
using sint8 = int8_t;
using uint16 = uint16_t;
using sint16 = int16_t;

namespace boost
{
inline bool iequals(std::string_view a, std::string_view b)
{
	if (a.size() != b.size())
		return false;
	for (size_t i = 0; i < a.size(); ++i)
		if (std::tolower(static_cast<unsigned char>(a[i])) != std::tolower(static_cast<unsigned char>(b[i])))
			return false;
	return true;
}
} // namespace boost

#include "input/motion/DSUMotion.h"
#include "input/motion/MotionHandler.h"

namespace
{
void Check(bool condition, const char* expression, int line)
{
	if (!condition)
		throw std::runtime_error(std::string("line ") + std::to_string(line) + ": " + expression);
}
#define CHECK(expression) Check((expression), #expression, __LINE__)

void test_calibration_cancel_and_failure_preserve_committed_bias()
{
	MahonySensorFusion imu;
	imu.setGyroBias(0.01f, -0.02f, 0.03f);
	imu.startGyroCalibration();
	for (int i = 0; i < 40; ++i)
		imu.updateIMU(0.01f, 0.2f, -0.1f, 0.15f, 0.0f, 0.0f, 1.0f);
	imu.cancelGyroCalibration();

	float bias[3]{};
	imu.getGyroBias(bias);
	CHECK(bias[0] == 0.01f && bias[1] == -0.02f && bias[2] == 0.03f);

	imu.startGyroCalibration();
	imu.updateIMU(0.01f, 0.2f, -0.1f, 0.15f, 0.0f, 0.0f, 1.0f);
	imu.updateIMU(0.01f, std::nanf(""), 0.0f, 0.0f, 0.0f, 0.0f, 1.0f);
	float progress = 0.0f;
	CHECK(imu.getGyroCalibrationState(progress) == GyroCalibrationState::Failed);
	imu.getGyroBias(bias);
	CHECK(bias[0] == 0.01f && bias[1] == -0.02f && bias[2] == 0.03f);
}

void test_calibration_does_not_reject_moving_samples()
{
	MahonySensorFusion imu;
	imu.startGyroCalibration();
	float progress = 0.0f;
	for (int i = 0; i < GyroCalibration::kSampleCount; ++i)
		imu.updateIMU(0.01f, 0.5f, -0.4f, 0.3f, 0.0f, 0.0f, 1.0f);

	CHECK(imu.getGyroCalibrationState(progress) == GyroCalibrationState::Success);
	CHECK(progress == 1.0f);
	float bias[3]{};
	imu.getGyroBias(bias);
	CHECK(bias[0] > 0.45f && bias[0] < 0.5f);
}

void test_passive_rest_tracking_boundary()
{
	MahonySensorFusion below;
	for (int i = 0; i < 1000; ++i)
		below.updateIMU(0.01f, 0.2f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f);
	float below_bias[3]{};
	below.getGyroBias(below_bias);
	CHECK(below_bias[0] > 0.0f);

	MahonySensorFusion above;
	for (int i = 0; i < 1000; ++i)
		above.updateIMU(0.01f, 0.4f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f);
	float above_bias[3]{};
	above.getGyroBias(above_bias);
	CHECK(above_bias[0] == 0.0f);
}

void test_threshold_and_sensitivity_output()
{
	float acceleration[3]{0.0f, 0.0f, 1.0f};
	float gyro[3]{0.2f, -0.1f, 0.05f};
	float orientation[3]{};
	float quaternion[4]{1.0f, 0.0f, 0.0f, 0.0f};
	MotionSample sample(acceleration, 0.0f, gyro, orientation, quaternion);

	float normal[3]{};
	float high[3]{};
	sample.getVPADGyroChange(normal, 1.0f, 0.0f);
	sample.getVPADGyroChange(high, 5.0f, 0.0f);
	for (int i = 0; i < 3; ++i)
		CHECK(std::abs(high[i] - normal[i] * 5.0f) < 0.00001f);

	float below_cutoff_accepted[3]{};
	float below_cutoff_filtered[3]{};
	float slow_gyro[3]{0.02f, 0.0f, 0.0f};
	MotionSample slow_sample(acceleration, 0.0f, slow_gyro, orientation, quaternion);
	slow_sample.getVPADGyroChange(below_cutoff_accepted, 1.0f, 0.0f);
	slow_sample.getVPADGyroChange(below_cutoff_filtered, 1.0f, Motion::kDefaultGyroThresholdTurnsPerSecond);
	CHECK(std::abs(below_cutoff_accepted[0]) > 0.0f);
	CHECK(below_cutoff_filtered[0] == 0.0f);
}

void test_dsu_scale_and_axis_mapping()
{
	float gyro[3]{};
	DSUMotion::MapGyro(312.0f, 624.0f, 936.0f, gyro);
	CHECK(std::abs(gyro[0] - 2.0f * 3.14159265f) < 0.00001f);
	CHECK(std::abs(gyro[1] - 6.0f * 3.14159265f) < 0.00001f);
	CHECK(std::abs(gyro[2] + 4.0f * 3.14159265f) < 0.00001f);

	float acceleration[3]{};
	DSUMotion::MapAcceleration(1.0f, 2.0f, 3.0f, acceleration);
	CHECK(acceleration[0] == 1.0f && acceleration[1] == -3.0f && acceleration[2] == 2.0f);
}
} // namespace

int main()
{
	try
	{
		test_calibration_cancel_and_failure_preserve_committed_bias();
		test_calibration_does_not_reject_moving_samples();
		test_passive_rest_tracking_boundary();
		test_threshold_and_sensitivity_output();
		test_dsu_scale_and_axis_mapping();
		std::cout << "All gyro checks passed.\n";
		return 0;
	}
	catch (const std::exception& error)
	{
		std::cerr << "Gyro check failed: " << error.what() << '\n';
		return 1;
	}
}
