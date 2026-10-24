#pragma once

#include <cmath>
#include "GyroCalibration.h"
#include "GyroThreshold.h"
#include "util/math/quaternion.h"

class MahonySensorFusion
{
public:
	MahonySensorFusion()
	{
		resetOrientation();
	}

	void resetOrientation()
	{
		// Default forward pose (holding controller forward, buttons/sticks facing up)
		m_imuQ.Assign(std::sqrt(0.5f), std::sqrt(0.5f), 0.0f, 0.0f);
		m_roll = 0.0f;
		m_pitch = 0.0f;
		m_yaw = 0.0f;
		m_rollWinding = 0;
		m_pitchWinding = 0;
		m_yawWinding = 0;
	}

	void startGyroCalibration()
	{
		m_calibration.start();
	}

	void cancelGyroCalibration()
	{
		m_calibration.cancel();
	}

	GyroCalibrationState getGyroCalibrationState(float& progress) const
	{
		return m_calibration.getState(progress);
	}

	void setGyroBias(float x, float y, float z)
	{
		m_calibration.setBias(x, y, z);
	}

	// gx, gy, gz are in radians/sec
	void updateIMU(float deltaTime, float gx, float gy, float gz, float ax, float ay, float az,
	                float gyroThreshold = Motion::kDefaultGyroThresholdTurnsPerSecond)
	{
		if (!std::isfinite(deltaTime) || !std::isfinite(gx) || !std::isfinite(gy) || !std::isfinite(gz) ||
		    !std::isfinite(ax) || !std::isfinite(ay) || !std::isfinite(az) || deltaTime <= 0.0f)
		{
			float progress = 0.0f;
			if (m_calibration.getState(progress) == GyroCalibrationState::Calibrating)
				m_calibration.fail();
			return;
		}

		// Safeguard against NaN / Inf / zero quaternion
		if (!std::isfinite(m_imuQ.w) || !std::isfinite(m_imuQ.x) || !std::isfinite(m_imuQ.y) || !std::isfinite(m_imuQ.z) ||
		    (m_imuQ.w * m_imuQ.w + m_imuQ.x * m_imuQ.x + m_imuQ.y * m_imuQ.y + m_imuQ.z * m_imuQ.z) < 0.0001f)
		{
			resetOrientation();
		}

		Vector3f av(ax, ay, az);
		Vector3f gv(gx, gy, gz);
		if (deltaTime > 0.2f)
			deltaTime = 0.2f;

		if (m_calibration.update(gx, gy, gz, ax, ay, az))
			resetOrientation();
		float gyroBias[3];
		m_calibration.getBias(gyroBias);
		gv.x -= gyroBias[0];
		gv.y -= gyroBias[1];
		gv.z -= gyroBias[2];

		if (Motion::IsGyroRateBelowThreshold(gv.x, gv.y, gv.z, gyroThreshold))
			gv = {};

		if (std::fabs(av.x) > 0.000001f || std::fabs(av.y) > 0.000001f || std::fabs(av.z) > 0.000001f)
		{
			av.Normalize();
			Vector3f grav = m_imuQ.GetVectorZ();
			grav.Scale(0.5f);
			Vector3f errorFeedback = grav.Cross(av);
			gv -= errorFeedback;
		}
		gv.Scale(0.5f * deltaTime);
		m_imuQ += (m_imuQ * Quaternionf(0.0f, gv.x, gv.y, gv.z));
		m_imuQ.NormalizeXYZW();
		updateOrientationAngles();
	}

	float getRollRadians() const
	{
		return m_roll + (float)m_rollWinding * 2.0f * 3.14159265f;
	}

	float getPitchRadians() const
	{
		return m_pitch + (float)m_pitchWinding * 2.0f * 3.14159265f;
	}

	float getYawRadians() const
	{
		return m_yaw + (float)m_yawWinding * 2.0f * 3.14159265f;
	}

	void getQuaternion(float q[4]) const
	{
		q[0] = m_imuQ.w;
		q[1] = m_imuQ.x;
		q[2] = m_imuQ.y;
		q[3] = m_imuQ.z;
	}

	void getGyroBias(float gBias[3]) const
	{
		m_calibration.getBias(gBias);
	}

	void getEulerAngles(float euler[3]) const
	{
		euler[0] = m_roll;
		euler[1] = m_pitch;
		euler[2] = m_yaw;
	}

private:
	// calculate roll, yaw and pitch in radians (-0.5 to 0.5)
	void calcOrientation()
	{
		float sinr_cosp = 2.0f * (m_imuQ.z * m_imuQ.w + m_imuQ.x * m_imuQ.y);
		float cosr_cosp = 1.0f - 2.0f * (m_imuQ.w * m_imuQ.w + m_imuQ.x * m_imuQ.x);
		m_roll = std::atan2(sinr_cosp, cosr_cosp);

		// pitch (y-axis rotation)
		float sinp = 2.0f * (m_imuQ.z * m_imuQ.x - m_imuQ.y * m_imuQ.w);
		if (std::abs(sinp) >= 1.0f)
			m_pitch = std::copysign(3.14159265359f / 2.0f, sinp);
		else
			m_pitch = std::asin(sinp);

		// yaw (z-axis rotation)
		float siny_cosp = 2.0f * (m_imuQ.z * m_imuQ.y + m_imuQ.w * m_imuQ.x);
		float cosy_cosp = 1.0f - 2.0f * (m_imuQ.x * m_imuQ.x + m_imuQ.y * m_imuQ.y);
		m_yaw = std::atan2(siny_cosp, cosy_cosp);
	}

	void updateOrientationAngles()
	{
		auto calcWindingCountChange = [](float prevAngle, float newAngle) -> int
		{
			if (newAngle > prevAngle)
			{
				float angleDif = newAngle - prevAngle;
				if (angleDif > 3.14159265f)
					return -1;
			}
			else if (newAngle < prevAngle)
			{
				float angleDif = prevAngle - newAngle;
				if (angleDif > 3.14159265f)
					return 1;
			}
			return 0;
		};
		float prevRoll = m_roll;
		float prevPitch = m_pitch;
		float prevYaw = m_yaw;
		calcOrientation();
		m_rollWinding += calcWindingCountChange(prevRoll, m_roll);
		m_pitchWinding += calcWindingCountChange(prevPitch, m_pitch);
		m_yawWinding += calcWindingCountChange(prevYaw, m_yaw);
	}

	Quaternionf m_imuQ; // current orientation
	// angle data
	float m_roll{};
	float m_pitch{};
	float m_yaw{};
	int m_rollWinding{};
	int m_pitchWinding{};
	int m_yawWinding{};
	GyroCalibration m_calibration;
};
