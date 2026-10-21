// The LG 360 VR (R100) headset, driven directly through hidapi.
//
// Ported from the vendored OpenHMD LGR100 driver
// (openhmd/src/drv_lgr100/lgr100.c, Joey Ferwerda, BSL-1.0) with the
// OpenHMD plumbing removed: the HID device is opened, read and fused here,
// and the results are handed to the SteamVR driver directly.

#include "lgr100_device.h"

#include <string.h>
#include <windows.h>
#include <stdlib.h>
#include <vector>

#include "lgr100_protocol.h"
#include "../driverlog.h"

// hidapi returns the descriptive strings as wide characters on Windows.
static std::string WideToUtf8(const wchar_t* w)
{
	if (!w)
	{
		return std::string();
	}
	int len = WideCharToMultiByte(CP_UTF8, 0, w, -1, NULL, 0, NULL, NULL);
	if (len <= 0)
	{
		return std::string();
	}
	std::vector<char> buf((size_t)len);
	WideCharToMultiByte(CP_UTF8, 0, w, -1, &buf[0], len, NULL, NULL);
	return std::string(&buf[0]);
}

// Measured tick rate (1872 ticks / 5.001 ms sample).
#define TICK_LEN (1.0f / 374325.0f)
#define TICK_FALLBACK 1872

// Raw gyro -> rad/s. Both axes converge to the same factor; yaw follows (no
// gravity reference exists for it).
#define GYRO_SCALE_X 1.05f // pitch
#define GYRO_SCALE_Y 1.05f // yaw
#define GYRO_SCALE_Z 1.05f // roll

// Continuous tracking of the gyro bias while the visor sits still.
//
// OFF by default: the bias is measured once at startup and re-measured only
// when the user asks for it (the OK button on the visor, which triggers
// recalibrateGyro()). The automatic tracking used to creep toward the
// actual motion whenever the head moved slowly, which silently cancelled
// part of the rotation. Set to 1 to get the old behaviour back.
#define LGR100_BIAS_AUTOTRACK 0

Lgr100Device::Lgr100Device()
	: m_handle(NULL)
	, m_running(false)
	, m_proximityPresent(false)
	, m_biasCount(0)
	, m_biasReady(false)
	, m_haveSample(false)
{
	m_vendor = "LG";
	m_product = "LGR100";

	memset(&m_orientation, 0, sizeof(m_orientation));
	m_orientation.w = 1.0f;
	memset(m_position, 0, sizeof(m_position));
	memset(m_buttons, 0, sizeof(m_buttons));
	m_gyroBias.x = m_gyroBias.y = m_gyroBias.z = 0.0f;
	memset(&m_sample, 0, sizeof(m_sample));
	m_lastTick = 0;
}

Lgr100Device::~Lgr100Device()
{
	close();
}

bool Lgr100Device::open()
{
	if (m_handle)
	{
		return true;
	}

	if (!openHandle())
	{
		return false;
	}

	// Wake the panels: without these commands they stay dark.
	sendActivation();

	m_running = true;
	m_thread = std::thread(&Lgr100Device::readerLoop, this);

	DriverLog("lgr100: opened %s\n", m_path.c_str());
	return true;
}

bool Lgr100Device::openHandle()
{
	if (m_handle)
	{
		return true;
	}

	// Find the headset's HID interface. The original driver took the first
	// interface the enumeration returns, so the same is done here.
	struct hid_device_info* devs = hid_enumerate(LGR100_VID, LGR100_PID);
	if (!devs)
	{
		m_error = "no LG 360 VR HID interface found";
		return false;
	}

	struct hid_device_info* pick = devs;

	m_handle = hid_open_path(pick->path);
	if (!m_handle)
	{
		m_error = "could not open the LG 360 VR HID interface";
		hid_free_enumeration(devs);
		return false;
	}

	m_path = pick->path ? pick->path : "";
	if (pick->manufacturer_string)
	{
		const std::string vendor = WideToUtf8(pick->manufacturer_string);
		if (!vendor.empty())
		{
			m_vendor = vendor;
		}
	}
	if (pick->product_string)
	{
		const std::string product = WideToUtf8(pick->product_string);
		if (!product.empty())
		{
			m_product = product;
		}
	}
	hid_free_enumeration(devs);

	// Non-blocking reads: the reader thread polls instead of parking on the
	// device, so it never holds up anything else.
	hid_set_nonblocking(m_handle, 1);

	// Sensor fusion. The stock gravity gain (0.05) corrects the tilt so slowly
	// that the gyro bias wins and the estimate drifts; 0.5 corrects it well
	// within a second without looking like a pose jump.
	ofusion_init(&m_fusion);
	m_fusion.grav_gain = 0.5f;

	return true;
}

void Lgr100Device::close()
{
	m_running = false;
	if (m_thread.joinable())
	{
		m_thread.join();
	}
	if (m_handle)
	{
		hid_close(m_handle);
		m_handle = NULL;
	}
}

void Lgr100Device::readerLoop()
{
	unsigned char buffer[FEATURE_BUFFER_SIZE];

	unsigned long long msgs = 0;
	unsigned long long samples = 0;
	unsigned long long lastLog = 0;

	while (m_running)
	{
		// Same shape as the original driver: a non-blocking read with a zero
		// timeout, polled in a tight loop. The device delivers the IMU stream
		// continuously, so there is always something to read.
		int size = hid_read_timeout(m_handle, buffer, FEATURE_BUFFER_SIZE, 0);
		if (size < 0)
		{
			// The headset was unplugged (or the endpoint died). Reopen it as
			// soon as it comes back, so a replug does not need a restart of
			// SteamVR.
			DriverLog("lgr100: read error, waiting for the headset to come back\n");
			if (m_handle)
			{
				hid_close(m_handle);
				m_handle = NULL;
			}
			while (m_running)
			{
				Sleep(1000);
				if (m_running && openHandle())
				{
					DriverLog("lgr100: headset reconnected, waking the panels\n");
					sendActivation();
					break;
				}
			}
			continue;
		}
		if (size == 0)
		{
			Sleep(1);
			continue;   // nothing pending
		}

		msgs++;
		if (msgs <= 5)
		{
			DriverLog("lgr100: report id %u, %d bytes\n", buffer[0], size);
		}
		if (buffer[0] == LGR100_IRQ_SENSORS)
		{
			samples++;
		}
		processReport(buffer, size);

		// A short heartbeat, so the log shows that the headset is alive and at
		// which rate the sensor stream arrives.
		const unsigned long long now = (unsigned long long)GetTickCount64();
		if (now - lastLog >= 10000)
		{
			lastLog = now;
			float q[4];
			getRotationQuat(q);
			DriverLog("lgr100: %llu msgs, %llu sensor samples, orient=(%.4f, %.4f, %.4f, %.4f)\n",
					  msgs, samples, q[0], q[1], q[2], q[3]);
		}
	}
}

void Lgr100Device::processReport(const unsigned char* buffer, int size)
{
	switch (buffer[0])
	{
	case LGR100_IRQ_NULL:
		break;

	case LGR100_IRQ_BUTTONS:
	{
		// button 'OK' is buffer[1] state 01 and 04
		// button '<-' is buffer[1] state 02 and 03
		int ok = -1;
		int back = -1;
		switch (buffer[1])
		{
		case LGR100_BUTTON_OK_ON:    ok = 1; break;
		case LGR100_BUTTON_BACK_ON:  back = 1; break;
		case LGR100_BUTTON_BACK_OFF: back = 0; break;
		case LGR100_BUTTON_OK_OFF:   ok = 0; break;
		default: break;
		}
		if (ok >= 0 || back >= 0)
		{
			std::lock_guard<std::mutex> lock(m_mutex);
			if (ok >= 0)
			{
				m_buttons[0] = ok;
			}
			if (back >= 0)
			{
				m_buttons[1] = back;
			}
		}
		break;
	}

	case LGR100_IRQ_DEBUG1:
	case LGR100_IRQ_DEBUG2:
	case LGR100_IRQ_DEBUG_SEQ1:
	case LGR100_IRQ_DEBUG_SEQ2:
		// The firmware floods these while in debug mode; they carry no data we
		// use, so they are dropped.
		break;

	case LGR100_IRQ_SENSORS:
		handleSensorMessage(buffer, size);
		break;

	case LGR100_IRQ_PROXIMITY:
		// The payload differs depending on whether something is in front of the
		// sensor. The byte that carries the state is byte 3: 0x00 when
		// something is in front, 0x14 when there is nothing.
		if (size >= 4)
		{
			std::lock_guard<std::mutex> lock(m_mutex);
			m_proximityPresent = (buffer[2] == 0x00);
		}
		break;

	case LGR100_IRQ_UNKNOWN2:
	case LGR100_IRQ_UNKNOWN3:
		break;

	default:
		break;
	}
}

void Lgr100Device::handleSensorMessage(const unsigned char* buffer, int size)
{
	uint32_t last_sample_tick = m_sample.tick;

	// Never fuse a sample we failed to decode: m_sample would still hold the
	// previous one and it would get integrated a second time.
	if (!decode_lgr100_imu_msg(&m_sample, buffer, size))
	{
		return;
	}

	// First sample (tick starts at ~0) or counter wrap -> fall back to a
	// nominal dt.
	uint32_t tick_delta = m_sample.tick - last_sample_tick;
	if (last_sample_tick == 0 || tick_delta == 0 || tick_delta > 100000)
	{
		tick_delta = TICK_FALLBACK;
	}

	float dt = tick_delta * TICK_LEN;

	vec3f raw_accel;
	raw_accel.x = m_sample.accel[0];
	raw_accel.y = m_sample.accel[1];
	raw_accel.z = -m_sample.accel[2];

	vec3f raw_gyro;
	raw_gyro.x = m_sample.gyro[0] * GYRO_SCALE_X;
	raw_gyro.y = m_sample.gyro[1] * GYRO_SCALE_Y;
	raw_gyro.z = m_sample.gyro[2] * GYRO_SCALE_Z;

	updateGyroBias(m_sample.gyro);

	raw_gyro.x -= m_gyroBias.x;
	raw_gyro.y -= m_gyroBias.y;
	raw_gyro.z -= m_gyroBias.z;

	vec3f mag = { { 0.0f, 0.0f, 0.0f } };

	std::lock_guard<std::mutex> lock(m_mutex);
	ofusion_update(&m_fusion, dt, &raw_gyro, &raw_accel, &mag);
	m_orientation = m_fusion.orient;
	m_lastTick = m_sample.tick;
	m_haveSample = true;
}

void Lgr100Device::updateGyroBias(const float* raw)
{
	float gx = raw[0] * GYRO_SCALE_X;
	float gy = raw[1] * GYRO_SCALE_Y;
	float gz = raw[2] * GYRO_SCALE_Z;

	float mag = sqrtf(gx * gx + gy * gy + gz * gz);

	if (!m_biasReady)
	{
		m_gyroBias.x += gx;
		m_gyroBias.y += gy;
		m_gyroBias.z += gz;
		m_biasCount++;
		// ~1 second of samples at 200 Hz
		if (m_biasCount >= 200)
		{
			m_gyroBias.x /= m_biasCount;
			m_gyroBias.y /= m_biasCount;
			m_gyroBias.z /= m_biasCount;
			m_biasReady = true;
			DriverLog("lgr100 gyro bias: %f %f %f\n", m_gyroBias.x, m_gyroBias.y, m_gyroBias.z);
		}
		return;
	}

#if LGR100_BIAS_AUTOTRACK
	// Only track the bias while the device is genuinely still. The threshold
	// has to sit well below a slow head turn (~0.5 rad/s), otherwise the
	// estimate creeps toward the actual motion and cancels the rotation out.
	if (mag < 0.04f)
	{
		m_gyroBias.x += (gx - m_gyroBias.x) * 0.002f;
		m_gyroBias.y += (gy - m_gyroBias.y) * 0.002f;
		m_gyroBias.z += (gz - m_gyroBias.z) * 0.002f;
	}
#else
	(void)mag;
#endif
}

bool Lgr100Device::recalibrateGyro()
{
	std::lock_guard<std::mutex> lock(m_mutex);
	m_biasReady = false;
	m_biasCount = 0;
	m_gyroBias.x = 0.0f;
	m_gyroBias.y = 0.0f;
	m_gyroBias.z = 0.0f;
	DriverLog("lgr100 gyro recalibration requested\n");
	return true;
}

bool Lgr100Device::activatePanels()
{
	return sendActivation();
}

bool Lgr100Device::sendActivation()
{
	if (!m_handle)
	{
		return false;
	}

	// The drain must stop on a read *timeout*, not on an empty read, and it
	// must not spin. With the headset awake the 200 Hz IMU stream never runs
	// dry, so a fixed count would stall the driver long enough to trip
	// SteamVR's watchdog. The overall time is bounded too, so a slow device
	// can never stall the driver.
	unsigned char drain_buf[FEATURE_BUFFER_SIZE];
	int ok = 0;

	for (int attempt = 0; attempt < 5 && !ok; attempt++)
	{
		DWORD deadline = GetTickCount() + 50;   // at most 50 ms of draining

		for (int d = 0; d < 64 && GetTickCount() < deadline; d++)
		{
			int n = hid_read_timeout(m_handle, drain_buf, FEATURE_BUFFER_SIZE, 5);
			if (n < 0)
			{
				break;                           // real error, stop
			}
			if (n == 0)
			{
				continue;                        // nothing pending, keep trying
			}
		}

		if (hid_write(m_handle, lgr100_cmd_sleep_disable, sizeof(lgr100_cmd_sleep_disable)) > 0 &&
			hid_write(m_handle, lgr100_cmd_vr_app_start, sizeof(lgr100_cmd_vr_app_start)) > 0)
		{
			ok = 1;
		}
		else if (attempt + 1 < 5)
		{
			Sleep(100);
		}
	}

	if (ok)
	{
		DriverLog("lgr100 activation commands sent\n");
	}
	else
	{
		m_error = "lgr100 activation command failed (hid_write rejected, is another process holding the HID?)";
		DriverLog("lgr100 activation command failed (hid_write rejected)\n");
	}
	return ok != 0;
}

void Lgr100Device::getRotationQuat(float out[4]) const
{
	std::lock_guard<std::mutex> lock(m_mutex);
	out[0] = m_orientation.x;
	out[1] = m_orientation.y;
	out[2] = m_orientation.z;
	out[3] = m_orientation.w;
}

void Lgr100Device::getPosition(float out[3]) const
{
	// No positional tracking on this headset: it is rotation only.
	out[0] = out[1] = out[2] = 0.0f;
}

void Lgr100Device::getButtons(float out[2]) const
{
	std::lock_guard<std::mutex> lock(m_mutex);
	out[0] = (float)m_buttons[0];
	out[1] = (float)m_buttons[1];
}

bool Lgr100Device::proximityPresent() const
{
	std::lock_guard<std::mutex> lock(m_mutex);
	return m_proximityPresent;
}

void Lgr100Device::getEyeProjection(int eye, float mat[16]) const
{
	// Same maths OpenHMD used (ohmd_calc_default_proj_matrices): the lenses
	// are not in the centre of the screen, so each eye's projection is offset.
	const float hsize = physicalWidth();
	const float lens_sep = lensSeparation();

	float screen_center = hsize / 4.0f;
	float lens_shift = screen_center - lens_sep / 2.0f;
	float proj_offset = fabs(4.0f * lens_shift / hsize);

	mat4x4f proj_base;
	omat4x4f_init_perspective(&proj_base, fov(), ratio(), znear(), zfar());

	mat4x4f translate;
	omat4x4f_init_translate(&translate, (eye == 0) ? proj_offset : -proj_offset, 0, 0);

	mat4x4f proj;
	omat4x4f_mult(&translate, &proj_base, &proj);

	// The panel's wiring rotates each eye's image by 90 degrees, so the
	// projection matrix is post-multiplied with that flip (the opposite
	// direction per eye). Without it the frustum comes out with the axes
	// swapped and the world renders stretched.
	mat4x4f flip;
	omat4x4f_init_ident(&flip);
	if (eye == 0)
	{
		// left eye
		flip.m[0][0] = 0.0f; flip.m[0][1] = 1.0f; flip.m[1][0] = -1.0f; flip.m[1][1] = 0.0f;
	}
	else
	{
		// right eye
		flip.m[0][0] = 0.0f; flip.m[0][1] = -1.0f; flip.m[1][0] = 1.0f; flip.m[1][1] = 0.0f;
	}
	omat4x4f_mult(&proj, &flip, &proj);

	// OpenVR expects the transposed matrix.
	omat4x4f_transpose(&proj, (mat4x4f*)mat);
}


