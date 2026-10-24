#pragma once

#include <wx/panel.h>
#include <functional>

#include "input/motion/GyroCalibration.h"
#include "input/motion/MotionSample.h"

class wxButton;
class wxGyroDraw;
class wxStaticText;

class wxGyroPanel : public wxPanel
{
public:
	wxGyroPanel(wxWindow* parent, wxWindowID id, const wxPoint& pos = wxDefaultPosition,
	            const wxSize& size = wxDefaultSize);

	void SetCalibrateCallback(std::function<void()> callback);
	void SetMotionSample(const MotionSample& sample, GyroCalibrationState state, float progress);
	void SetUnavailable();

private:
	void UpdateStatus(GyroCalibrationState state, float progress);

	wxGyroDraw* m_preview{};
	wxButton* m_calibrateButton{};
	wxStaticText* m_status{};
	std::function<void()> m_calibrateCallback;
};
