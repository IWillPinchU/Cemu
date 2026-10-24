#include "gui/components/wxGyroDraw.h"

#include <wx/dcbuffer.h>
#include <wx/graphics.h>
#include <wx/settings.h>
#include <cmath>
#include <array>
#include <algorithm>
#include <memory>

namespace {
struct Point2D {
	double x;
	double y;
};

struct Vec3f {
	float x, y, z;

	void RotateFromOrigin(float roll, float pitch, float yaw)
	{
		float temp = y;
		y = std::cos(roll) * y - std::sin(roll) * z;
		z = std::sin(roll) * temp + std::cos(roll) * z;

		temp = x;
		x = std::cos(pitch) * x + std::sin(pitch) * z;
		z = -std::sin(pitch) * temp + std::cos(pitch) * z;

		temp = x;
		x = std::cos(yaw) * x - std::sin(yaw) * y;
		y = std::sin(yaw) * temp + std::cos(yaw) * y;
	}
};
}

wxGyroDraw::wxGyroDraw(wxWindow* parent, wxWindowID id, const wxPoint& pos, const wxSize& size)
	: wxWindow(parent, id, pos, size, 0, wxPanelNameStr)
{
	SetBackgroundStyle(wxBG_STYLE_PAINT);
	SetToolTip(_("3D Gyroscope Preview"));
	Bind(wxEVT_PAINT, &wxGyroDraw::OnPaintEvent, this);
}

void wxGyroDraw::OnPaintEvent(wxPaintEvent&)
{
	wxAutoBufferedPaintDC dc(this);
	OnRender(dc);
}

void wxGyroDraw::SetOrientation(const MotionSample& sample)
{
	float next_euler[3];
	sample.getEulerAngles(next_euler);
	if (m_has_motion && std::equal(std::begin(m_euler), std::end(m_euler), std::begin(next_euler)))
		return;

	std::copy(std::begin(next_euler), std::end(next_euler), std::begin(m_euler));
	m_has_motion = true;
	Refresh(false);
}

void wxGyroDraw::OnRender(wxDC& dc)
{
	dc.SetBackgroundMode(wxSOLID);
	dc.SetBackground(wxSystemSettings::GetColour(wxSYS_COLOUR_WINDOW));
	dc.SetPen(wxSystemSettings::GetColour(wxSYS_COLOUR_WINDOW));
	dc.SetBrush(wxSystemSettings::GetColour(wxSYS_COLOUR_WINDOW));
	dc.Clear();

	const auto size = GetSize();
	const int min_dim = std::min(size.GetWidth(), size.GetHeight());
	const float half_w = size.GetWidth() / 2.0f;
	const float half_h = size.GetHeight() / 2.0f;

	auto graphics = std::unique_ptr<wxGraphicsContext>(wxGraphicsContext::CreateFromUnknownDC(dc));

	const auto border_color = wxSystemSettings::GetColour(wxSYS_COLOUR_GRAYTEXT);
	if (graphics)
	{
		// GraphicsContext keeps fractional coordinates and antialiases the moving edges.
		graphics->SetAntialiasMode(wxANTIALIAS_DEFAULT);
		graphics->SetPen(graphics->CreatePen(wxGraphicsPenInfo(border_color).Width(1.0)));
		graphics->SetBrush(*wxTRANSPARENT_BRUSH);
		graphics->DrawRectangle(0.5, 0.5, size.GetWidth() - 1.0, size.GetHeight() - 1.0);
	}
	else
	{
		dc.SetPen(border_color);
		dc.SetBrush(*wxTRANSPARENT_BRUSH);
		dc.DrawRectangle(0, 0, size.GetWidth() - 1, size.GetHeight() - 1);
	}

	// 8 canonical cube vertices matching Eden's Draw3dCube:
	// {-0.7f, -1.0f, -0.5f}, {-0.7f, 1.0f, -0.5f}, {0.7f, 1.0f, -0.5f}, {0.7f, -1.0f, -0.5f},
	// {-0.7f, -1.0f,  0.5f}, {-0.7f, 1.0f,  0.5f}, {0.7f, 1.0f,  0.5f}, {0.7f, -1.0f,  0.5f}
	std::array<Vec3f, 8> cube{
		Vec3f{-0.7f, -1.0f, -0.5f},
		Vec3f{-0.7f,  1.0f, -0.5f},
		Vec3f{ 0.7f,  1.0f, -0.5f},
		Vec3f{ 0.7f, -1.0f, -0.5f},
		Vec3f{-0.7f, -1.0f,  0.5f},
		Vec3f{-0.7f,  1.0f,  0.5f},
		Vec3f{ 0.7f,  1.0f,  0.5f},
		Vec3f{ 0.7f, -1.0f,  0.5f},
	};

	const float render_scale = min_dim * 0.32f;

	for (auto& pt : cube)
	{
		if (m_has_motion && IsEnabled())
		{
			pt.RotateFromOrigin(m_euler[0], m_euler[1], m_euler[2]);
		}
		pt.x *= render_scale;
		pt.y *= render_scale;
	}

	std::array<Point2D, 8> projected{};
	std::array<wxPoint, 8> fallback_projected{};
	for (size_t i = 0; i < cube.size(); ++i)
	{
		projected[i] = {half_w + cube[i].x, half_h + cube[i].y};
		fallback_projected[i] = {
			static_cast<int>(std::lround(projected[i].x)),
			static_cast<int>(std::lround(projected[i].y))};
	}

	// Match Eden's wireframe preview: one high-contrast outline, with no colored
	// face fills or separate front/back colors.
	const auto line_color = wxSystemSettings::GetColour(wxSYS_COLOUR_WINDOWTEXT);
	if (graphics)
		graphics->SetPen(graphics->CreatePen(wxGraphicsPenInfo(line_color).Width(1.5)));
	else
		dc.SetPen(wxPen(line_color, 2));
	for (int i = 0; i < 4; ++i)
	{
		const auto front = static_cast<size_t>(i);
		const auto back = front + 4;
		const auto next_front = static_cast<size_t>((i + 1) % 4);
		const auto next_back = next_front + 4;
		if (graphics)
		{
			graphics->StrokeLine(projected[front].x, projected[front].y,
			                     projected[next_front].x, projected[next_front].y);
			graphics->StrokeLine(projected[back].x, projected[back].y,
			                     projected[next_back].x, projected[next_back].y);
			graphics->StrokeLine(projected[front].x, projected[front].y,
			                     projected[back].x, projected[back].y);
		}
		else
		{
			dc.DrawLine(fallback_projected[front], fallback_projected[next_front]);
			dc.DrawLine(fallback_projected[back], fallback_projected[next_back]);
			dc.DrawLine(fallback_projected[front], fallback_projected[back]);
		}
	}
}
