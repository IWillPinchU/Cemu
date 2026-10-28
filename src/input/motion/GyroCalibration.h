#pragma once

#include <algorithm>
#include <cmath>
#include <iterator>


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
		std::copy(std::begin(m_bias), std::end(m_bias), std::begin(m_candidateBias));
		m_sampleCount = 0;
		m_progress = 0.0f;
		m_state = GyroCalibrationState::Calibrating;
	}

	void cancel()
	{
		discardCandidate();
		m_state = GyroCalibrationState::Idle;
	}

	void fail()
	{
		discardCandidate();
		m_state = GyroCalibrationState::Failed;
	}

	GyroCalibrationState getState(float& progress) const
	{
		progress = m_progress;
		return m_state;
	}

	// Sets the committed bias. If calibration is in progress, restarts the
	// calibration window using the new bias as the candidate baseline.
	void setBias(float x, float y, float z)
	{
		m_bias[0] = x;
		m_bias[1] = y;
		m_bias[2] = z;
		if (m_state == GyroCalibrationState::Calibrating)
			start();
	}

	void getBias(float bias[3]) const
	{
		bias[0] = m_bias[0];
		bias[1] = m_bias[1];
		bias[2] = m_bias[2];
	}

	// Returns true when the explicit calibration window completes. The candidate
	// bias becomes active only on success, so cancel/failure never leaves a partial
	// calibration behind. Samples are intentionally not rejected for movement.
	bool update(float gx, float gy, float gz, float ax, float ay, float az)
	{
		if (!std::isfinite(gx) || !std::isfinite(gy) || !std::isfinite(gz) || !std::isfinite(ax) ||
		    !std::isfinite(ay) || !std::isfinite(az))
		{
			if (m_state == GyroCalibrationState::Calibrating)
				fail();
			return false;
		}

		if (m_state == GyroCalibrationState::Calibrating)
		{
			// Eden-style EMA on every sample; no stillness/movement rejection.
			m_candidateBias[0] = m_candidateBias[0] * 0.99f + gx * 0.01f;
			m_candidateBias[1] = m_candidateBias[1] * 0.99f + gy * 0.01f;
			m_candidateBias[2] = m_candidateBias[2] * 0.99f + gz * 0.01f;

			++m_sampleCount;
			m_progress = static_cast<float>(m_sampleCount) / kSampleCount;
			if (m_sampleCount < kSampleCount)
				return false;

			std::copy(std::begin(m_candidateBias), std::end(m_candidateBias), std::begin(m_bias));
			m_state = GyroCalibrationState::Success;
			return true;
		}

		const float dx = gx - m_bias[0];
		const float dy = gy - m_bias[1];
		const float dz = gz - m_bias[2];
		const float gyroLength = std::sqrt(dx * dx + dy * dy + dz * dz);
		const float accelLength = std::sqrt(ax * ax + ay * ay + az * az);

		// Match Eden's slow background bias adaptation when the sample resembles rest.
		if (gyroLength < kRestThreshold && accelLength >= 0.90f && accelLength <= 1.10f)
		{
			m_bias[0] = m_bias[0] * 0.9999f + gx * 0.0001f;
			m_bias[1] = m_bias[1] * 0.9999f + gy * 0.0001f;
			m_bias[2] = m_bias[2] * 0.9999f + gz * 0.0001f;
		}

		return false;
	}

private:
	void discardCandidate()
	{
		std::copy(std::begin(m_bias), std::end(m_bias), std::begin(m_candidateBias));
		m_sampleCount = 0;
		m_progress = 0.0f;
	}

	static constexpr float kRestThreshold = 0.05f * 2.0f * 3.14159265f;
	float m_bias[3]{};
	float m_candidateBias[3]{};
	GyroCalibrationState m_state{GyroCalibrationState::Idle};
	float m_progress{};
	int m_sampleCount{};
};
