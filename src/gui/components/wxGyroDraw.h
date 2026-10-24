#pragma once

#include <wx/window.h>
#include "input/motion/MotionSample.h"

class wxGyroDraw : public wxWindow
{
public:
	wxGyroDraw(wxWindow* parent, wxWindowID id, const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxDefaultSize);
	void SetOrientation(const MotionSample& sample);

private:
	void OnPaintEvent(wxPaintEvent& event);
	void OnRender(wxDC& dc);

	float m_euler[3]{0.0f, 0.0f, 0.0f};
	bool m_has_motion{false};
};
