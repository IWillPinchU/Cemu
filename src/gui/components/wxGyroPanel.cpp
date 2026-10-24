#include "gui/components/wxGyroPanel.h"

#include <wx/button.h>
#include <wx/sizer.h>
#include <wx/stattext.h>
#include <utility>

#include "gui/components/wxGyroDraw.h"

wxGyroPanel::wxGyroPanel(wxWindow* parent, wxWindowID id, const wxPoint& pos, const wxSize& size, bool calibrationSupported)
	: wxPanel(parent, id, pos, size), m_calibrationSupported(calibrationSupported)
{
	auto* sizer = new wxBoxSizer(wxVERTICAL);
	m_preview = new wxGyroDraw(this, wxID_ANY, wxDefaultPosition, {60, 60});
	m_calibrateButton = new wxButton(this, wxID_ANY, _("Calibrate Gyro"));
	// Keep enough width for translated status text and reserve two lines so
	// calibration messages do not push outside the panel while it is updating.
	m_status = new wxStaticText(this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxSize{112, 34}, wxALIGN_CENTER);
	m_status->Wrap(112);

	sizer->Add(m_preview, 0, wxALIGN_CENTER | wxBOTTOM, 4);
	sizer->Add(m_calibrateButton, 0, wxALIGN_CENTER | wxBOTTOM, 2);
	sizer->Add(m_status, 0, wxALIGN_CENTER);
	SetSizer(sizer);
	sizer->SetSizeHints(this);

	m_calibrateButton->Bind(wxEVT_BUTTON, [this](wxCommandEvent&) {
		if (m_calibrateCallback)
			m_calibrateCallback();
	});
	SetUnavailable();
}

void wxGyroPanel::SetCalibrateCallback(std::function<void()> callback)
{
	m_calibrateCallback = std::move(callback);
}

void wxGyroPanel::SetMotionSample(const MotionSample& sample, GyroCalibrationState state, float progress)
{
	if (!m_preview->IsEnabled())
		m_preview->Enable();
	const bool can_calibrate = m_calibrationSupported && state != GyroCalibrationState::Calibrating;
	if (m_calibrateButton->IsEnabled() != can_calibrate)
		m_calibrateButton->Enable(can_calibrate);
	m_preview->SetOrientation(sample);
	UpdateStatus(state, progress);
}

void wxGyroPanel::SetUnavailable()
{
	if (m_preview->IsEnabled())
		m_preview->Disable();
	if (m_calibrateButton->IsEnabled())
		m_calibrateButton->Disable();
	m_status->SetLabelText(wxEmptyString);
}

void wxGyroPanel::UpdateStatus(GyroCalibrationState state, float progress)
{
	wxString label;
	switch (state)
	{
	case GyroCalibrationState::Calibrating:
		label = wxString::Format(_("Calibrating: %d%%"), static_cast<int>(progress * 100.0f));
		break;
	case GyroCalibrationState::Success:
		label = _("Calibrated successfully!");
		break;
	case GyroCalibrationState::Failed:
		label = _("Calibration stopped: invalid sensor data.");
		break;
	case GyroCalibrationState::Idle:
	default:
		label.clear();
		break;
	}
	if (m_status->GetLabelText() == label)
		return;
	m_status->SetLabelText(label);
	m_status->Wrap(112);
	Layout();
}
