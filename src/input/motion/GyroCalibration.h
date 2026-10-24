#pragma once

#include <cmath>

enum class GyroCalibrationState
{
	Idle,
	Calibrating,
	Success,
	Failed
};

// Tracks gyro bias and calibration progress independently of orientation fusion.
class GyroCalibration
{
public:
	static constexpr int kSampleCount = 300;

	void start()
	{
		m_sampleCount = 0;
		m_progress = 0.0f;
		m_state = GyroCalibrationState::Calibrating;
	}

	void cancel()
	{
		m_sampleCount = 0;
		m_progress = 0.0f;
		m_state = GyroCalibrationState::Idle;
	}

	void fail()
	{
		m_progress = 0.0f;
		m_state = GyroCalibrationState::Failed;
	}

	GyroCalibrationState getState(float& progress) const
	{
		progress = m_progress;
		return m_state;
	}

	void setBias(float x, float y, float z)
	{
		m_bias[0] = x;
		m_bias[1] = y;
		m_bias[2] = z;
	}

	void getBias(float bias[3]) const
	{
		bias[0] = m_bias[0];
		bias[1] = m_bias[1];
		bias[2] = m_bias[2];
	}

	// Returns true when the explicit calibration window completes.
	bool update(float gx, float gy, float gz, float ax, float ay, float az)
	{
		const float dx = gx - m_bias[0];
		const float dy = gy - m_bias[1];
		const float dz = gz - m_bias[2];
		const float gyroLength = std::sqrt(dx * dx + dy * dy + dz * dz);
		const float accelLength = std::sqrt(ax * ax + ay * ay + az * az);

		// Eden's slow background bias adaptation at rest.
		if (gyroLength < kRestThreshold && accelLength >= 0.90f && accelLength <= 1.10f)
		{
			m_bias[0] = m_bias[0] * 0.9999f + gx * 0.0001f;
			m_bias[1] = m_bias[1] * 0.9999f + gy * 0.0001f;
			m_bias[2] = m_bias[2] * 0.9999f + gz * 0.0001f;
		}

		if (m_state != GyroCalibrationState::Calibrating)
			return false;

		// Match Eden's explicit calibration update. It intentionally does not
		// reject movement; each sample contributes to the bias estimate.
		m_bias[0] = m_bias[0] * 0.99f + gx * 0.01f;
		m_bias[1] = m_bias[1] * 0.99f + gy * 0.01f;
		m_bias[2] = m_bias[2] * 0.99f + gz * 0.01f;

		++m_sampleCount;
		m_progress = static_cast<float>(m_sampleCount) / kSampleCount;
		if (m_sampleCount < kSampleCount)
			return false;

		m_state = GyroCalibrationState::Success;
		return true;
	}

private:
	static constexpr float kRestThreshold = 0.05f * 2.0f * 3.14159265f;
	float m_bias[3]{};
	GyroCalibrationState m_state{GyroCalibrationState::Idle};
	float m_progress{};
	int m_sampleCount{};
};
