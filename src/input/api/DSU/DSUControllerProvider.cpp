#include "input/api/DSU/DSUControllerProvider.h"
#include "input/api/DSU/DSUController.h"
#include "input/motion/GyroThreshold.h"
#include "input/motion/DSUMotion.h"

#include <algorithm>
#include <cmath>

#if BOOST_OS_WINDOWS
#include <boost/asio/detail/socket_option.hpp>
#include <winsock2.h>
#elif BOOST_OS_LINUX || BOOST_OS_MACOS
#include <sys/time.h>
#include <sys/socket.h>
#endif

DSUControllerProvider::DSUControllerProvider()
	: base_type(), m_uid(rand()), m_socket(m_io_service)
{
	for (auto& threshold : m_gyro_threshold)
		threshold.store(Motion::kDefaultGyroThresholdTurnsPerSecond, std::memory_order_relaxed);
	if (!connect())
	{
		throw std::runtime_error("dsu client can't open the udp connection");
	}

	m_running = true;
	m_reader_thread = std::thread(&DSUControllerProvider::reader_thread, this);
	m_writer_thread = std::thread(&DSUControllerProvider::writer_thread, this);
	request_version();
}

DSUControllerProvider::DSUControllerProvider(const DSUProviderSettings& settings)
	: base_type(settings), m_uid(rand()), m_socket(m_io_service)
{
	for (auto& threshold : m_gyro_threshold)
		threshold.store(Motion::kDefaultGyroThresholdTurnsPerSecond, std::memory_order_relaxed);
	if (!connect())
	{
		throw std::runtime_error("dsu client can't open the udp connection");
	}

	m_running = true;
	m_reader_thread = std::thread(&DSUControllerProvider::reader_thread, this);
	m_writer_thread = std::thread(&DSUControllerProvider::writer_thread, this);
	request_version();
}

DSUControllerProvider::~DSUControllerProvider()
{
	if (m_running)
	{
		m_running = false;
		m_writer_thread.join();
		m_reader_thread.join();
	}
}

std::vector<std::shared_ptr<ControllerBase>> DSUControllerProvider::get_controllers()
{
	std::vector<ControllerPtr> result;

	std::array<uint8_t, kMaxClients> indices;
	for (auto i = 0; i < kMaxClients; ++i)
		indices[i] = get_packet_index(i);

	request_pad_info();

	const auto controller_result = wait_update(indices, 3000);
	for (auto i = 0; i < kMaxClients; ++i)
	{
		if (controller_result[i] && is_connected(i))
			result.emplace_back(std::make_shared<DSUController>(i, m_settings));
	}

	return result;
}

bool DSUControllerProvider::connect()
{
	// already connected?
	if (m_receiver_endpoint.address().to_string() == get_settings().ip && m_receiver_endpoint.port() == get_settings().port)
		return true;

	try
	{
		using namespace boost::asio;

		ip::udp::resolver resolver(m_io_service);
		m_receiver_endpoint = *resolver.resolve(get_settings().ip, fmt::format("{}", get_settings().port)).cbegin();

		if (m_socket.is_open())
			m_socket.close();

		m_socket.open(ip::udp::v4());

        // set timeout for our threads to give a chance to exit
#if BOOST_OS_WINDOWS
        m_socket.set_option(boost::asio::detail::socket_option::integer<SOL_SOCKET, SO_RCVTIMEO>{200});
#elif BOOST_OS_LINUX || BOOST_OS_MACOS
        timeval timeout{.tv_usec = 200 * 1000};
        setsockopt(m_socket.native_handle(), SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeval));
#endif
		// Reset per-client state while holding the locks used by the reader and
		// motion paths. Never acquire both locks at once.
		for (size_t i = 0; i < kMaxClients; ++i)
		{
			{
				std::scoped_lock lock(m_mutex[i]);
				m_state[i] = {};
				m_prev_state[i] = {};
				m_last_motion_timestamp[i] = 0;
				m_last_motion_time[i] = {};
			}
		}

		// restart threads
		return true;
	}
	catch (const std::exception& ex)
	{
		cemuLog_log(LogType::Force, "dsu client connect error: {}", ex.what());
		return false;
	}
}

bool DSUControllerProvider::is_connected(uint8_t index) const
{
	if (index >= kMaxClients)
		return false;

	std::scoped_lock lock(m_mutex[index]);
	return m_state[index].info.state == DsState::Connected;
}

DSUControllerProvider::ControllerState DSUControllerProvider::get_state(uint8_t index) const
{
	if (index >= kMaxClients)
		return {};

	std::scoped_lock lock(m_mutex[index]);
	return m_state[index];
}

DSUControllerProvider::ControllerState DSUControllerProvider::get_prev_state(uint8_t index) const
{
	if (index >= kMaxClients)
		return {};

	std::scoped_lock lock(m_mutex[index]);
	return m_prev_state[index];
}

std::array<bool, DSUControllerProvider::kMaxClients> DSUControllerProvider::wait_update(
	const std::array<uint8_t, kMaxClients>& indices, size_t timeout) const
{
	std::array<bool, kMaxClients> result{false, false, false, false};

	const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout);
	do
	{
		for (int i = 0; i < kMaxClients; ++i)
		{
			if (result[i])
				continue;

			std::unique_lock lock(m_mutex[i]);
			result[i] = indices[i] < m_state[i].packet_index;
		}

		if (std::all_of(result.cbegin(), result.cend(), [](const bool& v) { return v == true; }))
			break;

		//std::this_thread::sleep_for(std::chrono::milliseconds(1));
		std::this_thread::yield();
	}
	while (std::chrono::steady_clock::now() < end);

	return result;
}

bool DSUControllerProvider::wait_update(uint8_t index, uint32_t packet_index, size_t timeout) const
{
	if (index >= kMaxClients)
		return false;

	std::unique_lock lock(m_mutex[index]);

	if (packet_index < m_state[index].packet_index)
		return true;

	const auto result = m_wait_cond[index].wait_for(lock, std::chrono::milliseconds(timeout),
	                                                [this, index, packet_index]()
	                                                {
		                                                return packet_index < m_state[index].packet_index;
	                                                });

	return result;
}

uint32_t DSUControllerProvider::get_packet_index(uint8_t index) const
{
	std::scoped_lock lock(m_mutex[index]);
	return m_state[index].packet_index;
}

void DSUControllerProvider::request_version()
{
	auto msg = std::make_unique<VersionRequest>(m_uid);

	std::scoped_lock lock(m_writer_mutex);
	m_writer_jobs.push(std::move(msg));
	m_writer_cond.notify_one();
}

void DSUControllerProvider::request_pad_info()
{
	auto msg = std::make_unique<ListPorts>(m_uid, 4, std::array<uint8_t, 4>{0, 1, 2, 3});

	std::scoped_lock lock(m_writer_mutex);
	m_writer_jobs.push(std::move(msg));
	m_writer_cond.notify_one();
}

void DSUControllerProvider::request_pad_info(uint8_t index)
{
	if (index >= kMaxClients)
		return;

	auto msg = std::make_unique<ListPorts>(m_uid, 1, std::array<uint8_t, 4>{index});

	std::scoped_lock lock(m_writer_mutex);
	m_writer_jobs.push(std::move(msg));
	m_writer_cond.notify_one();
}

void DSUControllerProvider::request_pad_data()
{
	auto msg = std::make_unique<DataRequest>(m_uid);

	std::scoped_lock lock(m_writer_mutex);
	m_writer_jobs.push(std::move(msg));
	m_writer_cond.notify_one();
}

void DSUControllerProvider::request_pad_data(uint8_t index)
{
	if (index >= kMaxClients)
		return;

	auto msg = std::make_unique<DataRequest>(m_uid, index);

	std::scoped_lock lock(m_writer_mutex);
	m_writer_jobs.push(std::move(msg));
	m_writer_cond.notify_one();
}

MotionSample DSUControllerProvider::get_motion_sample(uint8_t index) const
{
	if (index >= kMaxClients)
		return MotionSample();
	std::scoped_lock lock(m_mutex[index]);
	return m_state[index].motion_sample;
}

void DSUControllerProvider::start_gyro_calibration(uint8_t index)
{
	if (index >= kMaxClients)
		return;
	std::scoped_lock lock(m_mutex[index]);
	m_motion_handler[index].startGyroCalibration();
}

void DSUControllerProvider::cancel_gyro_calibration(uint8_t index)
{
	if (index >= kMaxClients)
		return;
	std::scoped_lock lock(m_mutex[index]);
	m_motion_handler[index].cancelGyroCalibration();
}

GyroCalibrationState DSUControllerProvider::get_gyro_calibration_state(uint8_t index, float& progress) const
{
	if (index >= kMaxClients)
	{
		progress = 0.0f;
		return GyroCalibrationState::Idle;
	}
	std::scoped_lock lock(m_mutex[index]);
	return m_motion_handler[index].getGyroCalibrationState(progress);
}

void DSUControllerProvider::set_gyro_bias(uint8_t index, float x, float y, float z)
{
	if (index >= kMaxClients)
		return;
	std::scoped_lock lock(m_mutex[index]);
	m_motion_handler[index].setGyroBias(x, y, z);
}

void DSUControllerProvider::get_gyro_bias(uint8_t index, float bias[3]) const
{
	if (index >= kMaxClients)
	{
		bias[0] = bias[1] = bias[2] = 0.0f;
		return;
	}
	std::scoped_lock lock(m_mutex[index]);
	m_motion_handler[index].getGyroBias(bias);
}

void DSUControllerProvider::set_gyro_threshold(uint8_t index, float threshold)
{
	if (index >= kMaxClients)
		return;
	if (!std::isfinite(threshold))
		threshold = Motion::kDefaultGyroThresholdTurnsPerSecond;
	m_gyro_threshold[index].store(
		std::clamp(threshold, 0.0f, Motion::kMaximumGyroThresholdTurnsPerSecond), std::memory_order_relaxed);
}

void DSUControllerProvider::reader_thread()
{
	SetThreadName("DSU-reader");
	bool first_read = true;
	while (m_running.load(std::memory_order_relaxed))
	{
		ServerMessage* msg;
		//try
		//{
		std::array<char, 100> recv_buf; // NOLINT(cppcoreguidelines-pro-type-member-init, hicpp-member-init)
		boost::asio::ip::udp::endpoint sender_endpoint;
		boost::system::error_code ec{};
		const size_t len = m_socket.receive_from(boost::asio::buffer(recv_buf), sender_endpoint, 0, ec);
		if (ec)
		{
#ifdef DEBUG_DSU_CLIENT
				printf(" DSUControllerProvider::ReaderThread: exception %s\n", ec.what());
#endif

			// there's probably no server listening on the given address:port
			if (first_read) // workaroud: first read always fails?
				first_read = false;
			else
			{
				std::this_thread::sleep_for(std::chrono::milliseconds(250));
				std::this_thread::yield();
			}
			continue;
		}

#ifdef DEBUG_DSU_CLIENT
			printf(" DSUControllerProvider::ReaderThread: received message with len: 0x%llx\n", len);
#endif

		if (len < sizeof(ServerMessage)) // cant be a valid message
			continue;

		msg = (ServerMessage*)recv_buf.data();
		//		}
		//		catch (const std::exception&)
		//		{
		//#ifdef DEBUG_DSU_CLIENT
		//			printf(" DSUControllerProvider::ReaderThread: exception %s\n", ex.what());
		//#endif
		//
		//			// there's probably no server listening on the given address:port
		//			if (first_read) // workaroud: first read always fails?
		//				first_read = false;
		//			else
		//			{
		//				std::this_thread::sleep_for(std::chrono::milliseconds(250));
		//				std::this_thread::yield();
		//			}
		//			continue;
		//		}

		uint8_t index = 0xFF;
		switch (msg->GetMessageType())
		{
		case MessageType::Version:
			{
				const auto rsp = (VersionResponse*)msg;
				if (!rsp->IsValid())
				{
#ifdef DEBUG_DSU_CLIENT
				printf(" DSUControllerProvider::ReaderThread: VersionResponse is invalid!\n");
#endif
					continue;
				}

#ifdef DEBUG_DSU_CLIENT
			printf(" DSUControllerProvider::ReaderThread: server version is: 0x%x\n", rsp->GetVersion());
#endif

				m_server_version = rsp->GetVersion();
				// wdc
				break;
			}
		case MessageType::Information:
			{
				const auto info = (PortInfo*)msg;
				if (!info->IsValid())
				{
#ifdef DEBUG_DSU_CLIENT
				printf(" DSUControllerProvider::ReaderThread: PortInfo is invalid!\n");
#endif
					continue;
				}

				index = info->GetIndex();
#ifdef DEBUG_DSU_CLIENT
			printf(" DSUControllerProvider::ReaderThread: received PortInfo for index %d\n", index);
#endif

				auto& mutex = m_mutex[index];
				std::scoped_lock lock(mutex);
				m_prev_state[index] = m_state[index];
				m_state[index] = *info;
				m_wait_cond[index].notify_all();
				break;
			}
		case MessageType::Data:
			{
				const auto rsp = (DataResponse*)msg;
				if (!rsp->IsValid())
				{
#ifdef DEBUG_DSU_CLIENT
				printf(" DSUControllerProvider::ReaderThread: DataResponse is invalid!\n");
#endif
					continue;
				}

				index = rsp->GetIndex();
#ifdef DEBUG_DSU_CLIENT
			printf(" DSUControllerProvider::ReaderThread: received DataResponse for index %d\n", index);
#endif

				auto& mutex = m_mutex[index];
				std::scoped_lock lock(mutex);
				m_prev_state[index] = m_state[index];
				m_state[index] = *rsp;
				m_wait_cond[index].notify_all();
				integrate_motion(index, *rsp);
				break;
			}
		}

		if (index != 0xFF)
			request_pad_data(index);
	}
}

void DSUControllerProvider::writer_thread()
{
	SetThreadName("DSU-writer");
	while (m_running.load(std::memory_order_relaxed))
	{
		std::unique_lock lock(m_writer_mutex);
		while (m_writer_jobs.empty())
		{
			if (m_writer_cond.wait_for(lock, std::chrono::milliseconds(250)) == std::cv_status::timeout)
			{
				if (!m_running.load(std::memory_order_relaxed))
					return;
			}
		}

		const auto msg = std::move(m_writer_jobs.front());
		m_writer_jobs.pop();
		lock.unlock();

#ifdef DEBUG_DSU_CLIENT
		printf(" DSUControllerProvider::WriterThread: sending message: 0x%x (len: 0x%x)\n", (int)msg->GetMessageType(), msg->GetSize());
#endif
		try
		{
			m_socket.send_to(boost::asio::buffer(msg.get(), msg->GetSize()), m_receiver_endpoint);
		}
		catch (const std::exception& ec)
		{
#ifdef DEBUG_DSU_CLIENT
			printf(" DSUControllerProvider::WriterThread: exception %s\n", ec.what());
#endif
			std::this_thread::sleep_for(std::chrono::milliseconds(250));
		}
	}
}

void DSUControllerProvider::integrate_motion(uint8_t index, const DataResponse& data_response)
{
	const uint64 ts = data_response.GetMotionTimestamp();
	float deltaTime = 0.01f;
	if (ts != 0)
	{
		if (ts <= m_last_motion_timestamp[index])
		{
			const uint64 dif = m_last_motion_timestamp[index] - ts;
			if (dif >= 10000000) // timestamp more than 10 seconds in the past, reset
				m_last_motion_timestamp[index] = 0;
			return;
		}
		const uint64 elapsedTime = ts - m_last_motion_timestamp[index];
		m_last_motion_timestamp[index] = ts;
		deltaTime = (float)((double)elapsedTime / 1000000.0);
		if (!std::isfinite(deltaTime) || deltaTime > 0.1f || deltaTime <= 0.0f)
			deltaTime = 0.01f;
	}
	else
	{
		const auto now = std::chrono::steady_clock::now();
		if (m_last_motion_time[index] != std::chrono::steady_clock::time_point{})
			deltaTime = std::chrono::duration<float>(now - m_last_motion_time[index]).count();
		m_last_motion_time[index] = now;

		if (!std::isfinite(deltaTime) || deltaTime > 0.1f || deltaTime <= 0.0f)
			deltaTime = 0.01f;
	}

	const auto& acc = data_response.GetAcceleration();
	const auto& gyro = data_response.GetGyro();
	// DSU rate scaling (degrees/second to radians/second) and native Cemu axis mapping.
	float mappedGyro[3];
	float mappedAcceleration[3];
	DSUMotion::MapGyro(gyro.x, gyro.y, gyro.z, mappedGyro);
	DSUMotion::MapAcceleration(acc.x, acc.y, acc.z, mappedAcceleration);
	m_motion_handler[index].processMotionSample(deltaTime,
	                                            mappedGyro[0],
	                                            mappedGyro[1],
	                                            mappedGyro[2],
	                                            mappedAcceleration[0],
	                                            mappedAcceleration[1],
	                                            mappedAcceleration[2],
	                                            m_gyro_threshold[index].load(std::memory_order_relaxed));
	m_state[index].motion_sample = m_motion_handler[index].getMotionSample();
}

DSUControllerProvider::ControllerState& DSUControllerProvider::ControllerState::operator=(const PortInfo& port_info)
{
	info = port_info.GetInfo();
	last_update = std::chrono::steady_clock::now();
	packet_index++; // increase packet index for every packet we assign/recv
	return *this;
}

DSUControllerProvider::ControllerState& DSUControllerProvider::ControllerState::operator=(
	const DataResponse& data_response)
{
	this->operator=(static_cast<const PortInfo&>(data_response));
	data = data_response.GetData();
	return *this;
}

