#define NOMINMAX
#include <windows.h>
#include <cstdint>
#include <filesystem>
#include <iostream>
#include <cassert>
#include <cmath>

using sint32 = int32_t;
using uint32 = uint32_t;
using uint64 = uint64_t;
using sint64 = int64_t;
using uint8 = uint8_t;
using sint8 = int8_t;
using uint16 = uint16_t;
using sint16 = int16_t;
namespace fs = std::filesystem;

namespace boost {
	inline bool iequals(std::string_view a, std::string_view b) {
		if (a.size() != b.size()) return false;
		for (size_t i = 0; i < a.size(); ++i) {
			if (tolower(a[i]) != tolower(b[i])) return false;
		}
		return true;
	}
}

#include "input/motion/Mahony.h"
#include "input/motion/MotionHandler.h"

void test_initial_state()
{
	MahonySensorFusion imu;
	float progress = 0.0f;
	assert(imu.getGyroCalibrationState(progress) == GyroCalibrationState::Idle);
	assert(progress == 0.0f);
	float bias[3]{};
	imu.getGyroBias(bias);
	assert(bias[0] == 0.0f && bias[1] == 0.0f && bias[2] == 0.0f);
	std::cout << "[PASS] test_initial_state\n";
}

void test_active_calibration_success()
{
	MahonySensorFusion imu;
	imu.startGyroCalibration();
	float progress = 0.0f;
	assert(imu.getGyroCalibrationState(progress) == GyroCalibrationState::Calibrating);

	const float test_bias_x = 0.045f;
	const float test_bias_y = -0.032f;
	const float test_bias_z = 0.018f;

	// Feed 300 stationary samples with gravity = 1.0g and constant gyro drift
	for (int i = 1; i <= 300; ++i)
	{
		imu.updateIMU(0.01f, test_bias_x, test_bias_y, test_bias_z, 0.0f, 0.0f, 1.0f);
		GyroCalibrationState state = imu.getGyroCalibrationState(progress);
		if (i < 300)
		{
			assert(state == GyroCalibrationState::Calibrating);
			assert(std::abs(progress - (float)i / 300.0f) < 0.01f);
		}
		else
		{
			assert(state == GyroCalibrationState::Success);
			assert(std::abs(progress - 1.0f) < 0.001f);
		}
	}

	float final_bias[3]{};
	imu.getGyroBias(final_bias);
	assert(std::abs(final_bias[0] - test_bias_x) < 0.0001f);
	assert(std::abs(final_bias[1] - test_bias_y) < 0.0001f);
	assert(std::abs(final_bias[2] - test_bias_z) < 0.0001f);

	std::cout << "[PASS] test_active_calibration_success (Bias: "
	          << final_bias[0] << ", " << final_bias[1] << ", " << final_bias[2] << ")\n";
}

void test_severe_drift_calibration()
{
	MahonySensorFusion imu;
	imu.startGyroCalibration();
	float progress = 0.0f;

	// Severe initial drift (e.g. 0.25 rad/s, which previously tripped the moved threshold)
	const float severe_drift = 0.25f;
	for (int i = 0; i < 300; ++i)
	{
		imu.updateIMU(0.01f, severe_drift, -severe_drift, severe_drift, 0.0f, 0.0f, 1.0f);
	}
	assert(imu.getGyroCalibrationState(progress) == GyroCalibrationState::Success);

	float final_bias[3]{};
	imu.getGyroBias(final_bias);
	assert(std::abs(final_bias[0] - severe_drift) < 0.005f);
	assert(std::abs(final_bias[1] - (-severe_drift)) < 0.005f);
	assert(std::abs(final_bias[2] - severe_drift) < 0.005f);

	std::cout << "[PASS] test_severe_drift_calibration (Converged severe drift accurately)\n";
}

void test_passive_resting_drift_tracking()
{
	MahonySensorFusion imu;
	// Initialize with a known baseline
	imu.setGyroBias(0.0200f, 0.0200f, 0.0200f);

	// When controller is stationary on desk (accel ~ 1.0g, gyro small):
	// Feed 1000 stationary samples with slightly shifted drift (0.0220f)
	for (int i = 0; i < 1000; ++i)
	{
		imu.updateIMU(0.01f, 0.0220f, 0.0220f, 0.0220f, 0.0f, 0.0f, 1.0f);
	}
	float adapted_bias[3]{};
	imu.getGyroBias(adapted_bias);
	// Should have adapted slightly towards 0.0220 without jumping
	assert(adapted_bias[0] > 0.0200f && adapted_bias[0] < 0.0220f);

	// Now simulate active gameplay aiming: gyro = 0.25 rad/s, accel = 1.3g
	float pre_aim_bias = adapted_bias[0];
	for (int i = 0; i < 500; ++i)
	{
		imu.updateIMU(0.01f, 0.25f, 0.15f, -0.10f, 0.3f, 0.2f, 1.25f);
	}
	float post_aim_bias[3]{};
	imu.getGyroBias(post_aim_bias);
	// Active gameplay aiming MUST NOT contaminate gyro bias
	assert(std::abs(post_aim_bias[0] - pre_aim_bias) < 0.00001f);

	std::cout << "[PASS] test_passive_resting_drift_tracking (Gameplay aiming did not corrupt bias)\n";
}

void test_motion_handler_integration()
{
	WiiUMotionHandler handler;
	float progress = 0.0f;
	assert(handler.getGyroCalibrationState(progress) == GyroCalibrationState::Idle);

	handler.startGyroCalibration();
	assert(handler.getGyroCalibrationState(progress) == GyroCalibrationState::Calibrating);

	const float drift_x = 0.035f;
	const float drift_y = -0.025f;
	const float drift_z = 0.015f;

	for (int i = 0; i < 300; ++i)
	{
		handler.processMotionSample(0.01f, drift_x, drift_y, drift_z, 0.0f, 0.0f, 1.0f);
	}
	assert(handler.getGyroCalibrationState(progress) == GyroCalibrationState::Success);

	// MotionSample debiased gyro should now be close to 0
	MotionSample sample = handler.getMotionSample();
	float vpad_gyro[3]{};
	sample.getVPADGyroChange(vpad_gyro);
	assert(std::abs(vpad_gyro[0]) < 0.001f);
	assert(std::abs(vpad_gyro[1]) < 0.001f);
	assert(std::abs(vpad_gyro[2]) < 0.001f);

	std::cout << "[PASS] test_motion_handler_integration (Debiased VPAD output zeroed)\n";
}

int main()
{
	std::cout << "Running Gyroscope Calibration Unit Tests...\n";
	test_initial_state();
	test_active_calibration_success();
	test_severe_drift_calibration();
	test_passive_resting_drift_tracking();
	test_motion_handler_integration();
	std::cout << "All 5 unit tests passed successfully!\n";
	return 0;
}
