// The LG 360 VR (R100) headset, driven directly through hidapi.
//
// This is the standalone replacement for the vendored OpenHMD stack: the
// driver owns the HID device, decodes the IMU reports, runs the sensor
// fusion and exposes the panel geometry. Nothing talks to OpenHMD anymore,
// so the whole thing is a single block that compiles once.

#ifndef LGR100_DEVICE_H
#define LGR100_DEVICE_H

#include <atomic>
#include <mutex>
#include <string>
#include <thread>

#include <hidapi.h>

#include "lgr100_fusion.h"
#include "lgr100_omath.h"
#include "lgr100_protocol.h"

class Lgr100Device
{
public:
	Lgr100Device();
	~Lgr100Device();

	// Finds the headset's HID interface and starts the reader thread.
	bool open();
	void close();

	// --- identity -------------------------------------------------------
	const std::string& vendor() const  { return m_vendor; }
	const std::string& product() const { return m_product; }
	const std::string& path() const    { return m_path; }

	// --- panel geometry -------------------------------------------------
	// The panel is two 960x720 panels side by side, which Windows surfaces as
	// a single 1440x960 display (each eye gets a 720x960 portrait slot).
	int   screenWidth() const           { return 1440; }
	int   screenHeight() const          { return 960; }
	float physicalWidth() const         { return 0.110f; }
	float physicalHeight() const        { return 0.038f; }
	float ipd() const                   { return 0.061f; }
	float fov() const                   { return DEG_TO_RAD(88.0f); }
	float ratio() const                 { return 720.0f / 960.0f; }
	float lensSeparation() const        { return 0.0635f; }
	float lensVerticalPosition() const { return 0.020f; }
	float znear() const                 { return 0.1f; }
	float zfar() const                  { return 1000.0f; }

	// Projection matrix of one eye, transposed the way OpenVR expects it
	// (same maths OpenHMD used, so the rendering does not change).
	void getEyeProjection(int eye, float mat[16]) const;

	// --- live state -----------------------------------------------------
	void getRotationQuat(float out[4]) const;   // x, y, z, w
	void getPosition(float out[3]) const;
	void getButtons(float out[2]) const;        // OK, BACK
	bool proximityPresent() const;

	// --- commands --------------------------------------------------------
	// Restart the gyro bias estimation: the next ~200 samples are used to
	// measure the gyro DC offset again.
	bool recalibrateGyro();
	// Re-send "Sleep Disable" + "VR App Start" so the panels light up again.
	bool activatePanels();

	const char* lastError() const { return m_error.c_str(); }

private:
	Lgr100Device(const Lgr100Device&);
	Lgr100Device& operator=(const Lgr100Device&);

	void readerLoop();
	void processReport(const unsigned char* buffer, int size);
	void handleSensorMessage(const unsigned char* buffer, int size);
	void updateGyroBias(const float* raw);
	bool sendActivation();

	hid_device* m_handle;
	std::thread m_thread;
	std::atomic<bool> m_running;

	mutable std::mutex m_mutex;      // guards the state below
	quatf m_orientation;
	float m_position[3];
	int m_buttons[2];
	bool m_proximityPresent;

	// Gyro bias estimation.
	vec3f m_gyroBias;
	int m_biasCount;
	bool m_biasReady;

	// Live sample state.
	lgr100_sensor_sample m_sample;
	uint32_t m_lastTick;
	bool m_haveSample;

	fusion m_fusion;

	std::string m_vendor;
	std::string m_product;
	std::string m_path;
	std::string m_error;
};

#endif
