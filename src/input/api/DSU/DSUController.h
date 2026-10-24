#pragma once

#include "input/api/Controller.h"
#include "input/api/DSU/DSUControllerProvider.h"

class DSUController : public Controller<DSUControllerProvider>
{
public:
	DSUController(uint32 index);
	DSUController(uint32 index, const DSUProviderSettings& settings);
	
	std::string_view api_name() const override
	{
		static_assert(to_string(InputAPI::DSUClient) == "DSUController");
		return to_string(InputAPI::DSUClient);
	}
	InputAPI::Type api() const override { return InputAPI::DSUClient; }

	void save(pugi::xml_node& node) override;
	void load(const pugi::xml_node& node) override;
	void set_settings(const ControllerBase::Settings& settings) override;

	bool connect() override;
	bool is_connected() override;
	
	bool has_motion() override { return true; }
	MotionSample get_motion_sample() override;

	void start_gyro_calibration() override;
	void cancel_gyro_calibration() override;
	GyroCalibrationState get_gyro_calibration_state(float& progress) const override;
	void set_gyro_bias(float x, float y, float z) override;
	void get_gyro_bias(float bias[3]) const override;

	bool has_position() override;
	glm::vec2 get_position() override;
	glm::vec2 get_prev_position() override;
	PositionVisibility GetPositionVisibility() override;

	std::string get_button_name(uint64 button) const override;

protected:
	ControllerState raw_state() override;

private:
	uint32 m_index;
};

