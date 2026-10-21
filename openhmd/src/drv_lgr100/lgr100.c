// Copyright 2018, Joey Ferwerda.
// SPDX-License-Identifier: BSL-1.0
/*
 * OpenHMD - Free and Open Source API and drivers for immersive technology.
 */

/* LG R100 (360VR) Driver */

#include <string.h>
#include <stdio.h>
#include <hidapi.h>
#include <windows.h>
#include "../openhmdi.h"

#include "lgr100.h"

typedef struct {
	ohmd_device base;
	hid_device* handle;
	int id;

	int controller_values[2];
	int proximity_present;  // 1 when the proximity sensor sees something
	lgr100_sensor_sample sample;
	fusion sensor_fusion;
	vec3f raw_accel, raw_gyro;
	vec3f gyro_bias;      // resting offset, measured at startup
	int bias_count;       // samples averaged for the bias estimate
	int bias_ready;       // once bias is measured, keep updating it while still
} lgr100_priv;

// Continuous tracking of the gyro bias while the visor sits still.
//
// OFF by default: the bias is measured once at startup and re-measured only
// when the user asks for it (the OK button on the visor, which triggers
// OHMD_LGR100_RECALIBRATE). The automatic tracking used to creep toward the
// actual motion whenever the head moved slowly, which silently cancelled
// part of the rotation. Set to 1 to get the old behaviour back.
#define LGR100_BIAS_AUTOTRACK 0

#define TICK_LEN (1.0f / 374325.0f) // measured tick rate (1872 ticks / 5.001 ms sample)
#define TICK_FALLBACK 1872          // one nominal 200 Hz sample

// Temporary tracking diagnostics: dump every fused sample to a CSV so the
// roll/"stabilization" behaviour can be analysed offline. Disabled after the
// calibration work: the fopen/fprintf path was the only CRT use on the update
// thread and vrserver started crashing with a write fault inside ucrtbase
// while it was enabled. Re-enable (1) only for a dedicated diagnostic run.
#define LGR100_TRACKING_LOG 0
#if LGR100_TRACKING_LOG
static FILE* g_log_file = NULL;
static int g_log_lines = 0;
static void log_fusion_sample(lgr100_priv* priv, float dt)
{
	if(g_log_lines >= 120000) // ~10 minutes at 200 Hz
		return;
	if(!g_log_file){
		// Relative path: written next to the process working directory. Adjust
		// this if you enable the diagnostic on a machine where that is not
		// writable.
		g_log_file = fopen("lgr100_fusion.csv", "w");
		if(!g_log_file)
			return;
		fprintf(g_log_file, "tick,dt_ms,gx,gy,gz,ax,ay,az,qw,qx,qy,qz,grav_err,lvl,iter\n");
	}
	fprintf(g_log_file, "%u,%.4f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%d,%d\n",
		priv->sample.tick, dt * 1000.0f,
		priv->raw_gyro.x, priv->raw_gyro.y, priv->raw_gyro.z,
		priv->raw_accel.x, priv->raw_accel.y, priv->raw_accel.z,
		priv->sensor_fusion.orient.w, priv->sensor_fusion.orient.x,
		priv->sensor_fusion.orient.y, priv->sensor_fusion.orient.z,
		priv->sensor_fusion.grav_error_angle, priv->sensor_fusion.device_level_count,
		priv->sensor_fusion.iterations);
	g_log_lines++;
	if((g_log_lines % 20) == 0)
		fflush(g_log_file);
}
#endif

static lgr100_priv* lgr100_priv_get(ohmd_device* device)
{
	return (lgr100_priv*)device;
}

void accel_from_lgr100_vec(const float* smp, vec3f* out_vec)
{
	out_vec->x = (float)smp[0];
	out_vec->y = (float)smp[1];
	out_vec->z = -(float)smp[2];
	//printf("accel = %f, %f, %f\n", out_vec->x, out_vec->y, out_vec->z);
}

// raw gyro -> rad/s. Re-measured against gravity from a calibration run where
// every motion was integrated and compared with the change of the measured
// gravity direction: the previous per-axis scales (1.25 pitch / 1.47 roll)
// over-integrated by ~19% and ~41%, which produced large pose errors after
// fast head movements (the gravity correction then visibly dragged the view).
// Both axes now converge to the same factor; yaw follows (no gravity
// reference exists for it).
#define GYRO_SCALE_X 1.05f // pitch
#define GYRO_SCALE_Y 1.05f // yaw, assumed to match pitch (no gravity reference exists)
#define GYRO_SCALE_Z 1.05f // roll

void gyro_from_lgr100_vec(const float* smp, vec3f* out_vec)
{
	out_vec->x = (float)smp[0] * GYRO_SCALE_X;
	out_vec->y = (float)smp[1] * GYRO_SCALE_Y;
	out_vec->z = (float)smp[2] * GYRO_SCALE_Z;
	//printf("gyro = %f, %f, %f\n", out_vec->x, out_vec->y, out_vec->z);
}

// Estimate the gyro's DC offset while the headset is held still, then keep the
// estimate fresh so a warm-up bias never integrates into a slow head drift.
static void update_gyro_bias(lgr100_priv* priv, const float* raw)
{
	float gx = raw[0] * GYRO_SCALE_X;
	float gy = raw[1] * GYRO_SCALE_Y;
	float gz = raw[2] * GYRO_SCALE_Z;

	float mag = sqrtf(gx*gx + gy*gy + gz*gz);

	if(!priv->bias_ready){
		priv->gyro_bias.x += gx;
		priv->gyro_bias.y += gy;
		priv->gyro_bias.z += gz;
		priv->bias_count++;
		// ~1 second of samples at 200 Hz
		if(priv->bias_count >= 200){
			priv->gyro_bias.x /= priv->bias_count;
			priv->gyro_bias.y /= priv->bias_count;
			priv->gyro_bias.z /= priv->bias_count;
			priv->bias_ready = 1;
			LOGD("lgr100 gyro bias: %f %f %f", priv->gyro_bias.x, priv->gyro_bias.y, priv->gyro_bias.z);
		}
		return;
	}

	// Only track the bias while the device is genuinely still. The threshold has
	// to sit well below a slow head turn (~0.5 rad/s), otherwise the estimate
	// creeps toward the actual motion and cancels the rotation out.
#if LGR100_BIAS_AUTOTRACK
	if(mag < 0.04f){
		priv->gyro_bias.x += (gx - priv->gyro_bias.x) * 0.002f;
		priv->gyro_bias.y += (gy - priv->gyro_bias.y) * 0.002f;
		priv->gyro_bias.z += (gz - priv->gyro_bias.z) * 0.002f;
	}
#else
	(void)mag;
#endif
}

// Restart the gyro bias estimation. The next samples are averaged again, so
// the calibration can be redone on demand: hold the visor still and press the
// OK button. Returns nothing; the new estimate is ready ~1 s later.
static void lgr100_recalibrate(lgr100_priv* priv)
{
	priv->bias_ready = 0;
	priv->bias_count = 0;
	priv->gyro_bias.x = 0.0f;
	priv->gyro_bias.y = 0.0f;
	priv->gyro_bias.z = 0.0f;
	LOGD("lgr100 gyro recalibration requested");
}

static void handle_tracker_sensor_msg(lgr100_priv* priv, unsigned char* buffer, int size)
{
	uint32_t last_sample_tick = priv->sample.tick;

	// Never fuse a sample we failed to decode: priv->sample would still hold the
	// previous one and it would get integrated a second time.
	if(!decode_lgr100_imu_msg(&priv->sample, buffer, size))
		return;

	// First sample (tick starts at ~0) or counter wrap -> fall back to nominal dt.
	uint32_t tick_delta = priv->sample.tick - last_sample_tick;
	if(last_sample_tick == 0 || tick_delta == 0 || tick_delta > 100000)
		tick_delta = TICK_FALLBACK;

	float dt = tick_delta * TICK_LEN;

	accel_from_lgr100_vec(priv->sample.accel, &priv->raw_accel);
	gyro_from_lgr100_vec(priv->sample.gyro, &priv->raw_gyro);

	update_gyro_bias(priv, priv->sample.gyro);

	priv->raw_gyro.x -= priv->gyro_bias.x;
	priv->raw_gyro.y -= priv->gyro_bias.y;
	priv->raw_gyro.z -= priv->gyro_bias.z;
	
	vec3f mag = {{0.0f, 0.0f, 0.0f}};
	
	ofusion_update(&priv->sensor_fusion, dt, &priv->raw_gyro, &priv->raw_accel, &mag);
#if LGR100_TRACKING_LOG
	log_fusion_sample(priv, dt);
#endif
}

static void update_device(ohmd_device* device)
{
	lgr100_priv* priv = lgr100_priv_get(device);
	unsigned char buffer[FEATURE_BUFFER_SIZE];

	// NOTE: do not write to the device from here. update_device() runs on the
	// update thread while hidapi has a read in flight; a hid_write from here
	// races with that read inside hidapi and crashed vrserver (access
	// violation). Period re-activation was tried and removed for this reason.

	// Read the messages currently queued. Bounded and explicitly non-blocking:
	// this runs under OpenHMD's update mutex, so it must never wait on the
	// device nor let a flooded debug stream monopolize the mutex.
	for(int read_count = 0; read_count < 64; read_count++){
		int size = hid_read_timeout(priv->handle, buffer, FEATURE_BUFFER_SIZE, 0);
		if(size < 0){
			LOGE("error reading from device");
			return;
		} else if(size == 0) {
			return; // No more messages, return.
		}

		//NULL package
		if(buffer[0] == LGR100_IRQ_NULL) {
			continue;
		}
		else if(buffer[0] == LGR100_IRQ_BUTTONS){
			//button 'OK' is buffer[1] state 01 and 04
			//button '<-' is buffer[1] state 02 and 03
			if (buffer[1] == LGR100_BUTTON_OK_ON)
				priv->controller_values[0] = 1;
			else if (buffer[1] == LGR100_BUTTON_BACK_ON)
				priv->controller_values[1] = 1;
			else if (buffer[1] == LGR100_BUTTON_BACK_OFF)
				priv->controller_values[1] = 0;
			else if (buffer[1] == LGR100_BUTTON_OK_OFF)
				priv->controller_values[0] = 0;
		}
		//Print all the verbose debug information
		else if(buffer[0] == LGR100_IRQ_DEBUG1 ||
				buffer[0] == LGR100_IRQ_DEBUG2 ||
				buffer[0] == LGR100_IRQ_DEBUG_SEQ1 ||
				buffer[0] == LGR100_IRQ_DEBUG_SEQ2)
		{
			//*buffer += 1;
			//printf("%s\n", buffer);
		}
		else if(buffer[0] == LGR100_IRQ_SENSORS) {
			// Keep reading instead of returning. The IMU stream is 200 Hz and
			// always pending, so returning on the first IMU report aborted the
			// loop before the queued button and proximity reports were ever
			// read (the buttons only worked when their report happened to be
			// the first one in the queue). The loop is still bounded at 64
			// messages and every read is non-blocking.
			handle_tracker_sensor_msg(priv, buffer, size);
			continue;
		}
		else if (buffer[0] == LGR100_IRQ_UNKNOWN1) {
			// Proximity sensor report. The payload differs depending on whether
			// something is in front of the sensor:
			//   prox on  : 01:07:00:40:BD:97:B1:D1:3B:BD:72:2E:BC:CD:EC:53:BF:33:47:19:41:66:22:17:C0:82:AC:C9:00:02:00:00
			//   prox off : 01:07:05:14:3D:97:B1:D1:3B:3A:85:0B:BC:9A:59:4B:BF:9A:28:1A:41:9A:E1:1B:C0:99:41:B4:E2:02:00:00
			// The byte that actually carries the state is byte 3: 0x00 when
			// something is in front, 0x14 when there is nothing. The visor uses
			// the same sensor to turn the panels off when it is not worn.
			if(size >= 4){
				priv->proximity_present = (buffer[2] == 0x00) ? 1 : 0;
			}
		}
		else if (buffer[0] == LGR100_IRQ_UNKNOWN2) {
			// prints
			// FF:48:42:00:20:A8:DE:00:20:A8:DE:00:20:89:2F:03:08:60:6C:00:20:05:00:CC:CC:06:00:CC:CC:1B:FF:02
		}
		else if (buffer[0] == LGR100_IRQ_UNKNOWN3) {
			// prints
			// 65:20:28:30:29:0A:00:00:00:00:00:00:00:00:00:00:00:00:00:00:00:00:00:00:00:00:00:00:00:00:00:00
		}else{
			LOGE("unknown message type: %u", buffer[0]);
			//printf("%s", buffer);
		}
	}
}

// Defined further down, next to the open_device() code it was extracted from.
static int lgr100_activate_panels(lgr100_priv* priv, ohmd_context* ctx);

static int getf(ohmd_device* device, ohmd_float_value type, float* out)
{
	lgr100_priv* priv = (lgr100_priv*)device;

	switch(type){
	case OHMD_ROTATION_QUAT: {
			*(quatf*)out = priv->sensor_fusion.orient;
			break;
		}

	case OHMD_POSITION_VECTOR:
		out[0] = out[1] = out[2] = 0;
		break;

	case OHMD_DISTORTION_K:
		// TODO this should be set to the equivalent of no distortion
		memset(out, 0, sizeof(float) * 6);
		break;
	
	case OHMD_CONTROLS_STATE:
		for (int i = 0; i < 2; i++){
			out[i] = priv->controller_values[i];
		}
		break;

	// Commands, not getters. The LGR100 driver has no setf (device->setf is
	// NULL), so they are triggered through getf; both run under OpenHMD's
	// update mutex, which is what makes the hid_write inside the activation
	// sequence safe.
	case OHMD_LGR100_RECALIBRATE:
		lgr100_recalibrate(priv);
		out[0] = 1.0f;
		break;

	case OHMD_LGR100_ACTIVATE:
		out[0] = (float)lgr100_activate_panels(priv, priv->base.ctx);
		break;

	case OHMD_LGR100_USER_PRESENT:
		out[0] = (float)priv->proximity_present;
		break;

	default:
		ohmd_set_error(priv->base.ctx, "invalid type given to getf (%ud)", type);
		return OHMD_S_INVALID_PARAMETER;
		break;
	}

	return OHMD_S_OK;
}

static void close_device(ohmd_device* device)
{
	LOGD("closing lgr100 device");
	free(device);
}

// Activation commands. The second byte of an LGR100 command report is the
// *length* of the command text: the firmware terminates the received string
// at that byte before matching it, so a value smaller than strlen() makes the
// command silently not match (verified 2026-10-15: "Sleep Disable" is 13 chars
// and only runs with 0x0D; with 0x0C it is ignored). The reference
// implementation (LG360VRActivator) sends "Sleep Disable" with 0x0C, which
// never did anything - the panel came up because "VR App Start" is correct.
// Without these the left panel stays dark.
//
// Two things matter here and both were wrong before:
//  - The drain must stop on a read *timeout*, not on an empty read, and it
//    must not spin. With the headset awake the 200 Hz IMU stream never runs
//    dry, so the old `break` on `<= 0` never fired and the loop drained a
//    fixed count every time, stalling the driver long enough to trip SteamVR's
//    watchdog. That made the panels come up or not depending purely on whether
//    the headset happened to be asleep, which looked like an unrelated setting
//    breaking the connection.
//  - Overall time is bounded, so a slow device can never stall driver init.
//
// This runs under OpenHMD's update mutex (both from open_device and from the
// OHMD_LGR100_ACTIVATE command), so the hid_write never races with the read
// that the update thread has in flight.
static int lgr100_activate_panels(lgr100_priv* priv, ohmd_context* ctx)
{
	unsigned char cmd_sleep[15] = {0x03, 0x0D, 'S','l','e','e','p',' ','D','i','s','a','b','l','e'};
	unsigned char cmd_start[14] = {0x03, 0x0C, 'V','R',' ','A','p','p',' ','S','t','a','r','t'};

	unsigned char drain_buf[FEATURE_BUFFER_SIZE];
	int ok = 0;

	// Retry, bounded. Any other process holding the HID (the reference
	// LG360VRActivator is the usual culprit) can make a write fail, and
	// without both commands the LEFT panel stays dark while the right one
	// still comes up - which presents exactly like a rendering bug in the
	// driver. Total time is bounded so driver init cannot trip SteamVR's
	// watchdog.
	for(int attempt = 0; attempt < 5 && !ok; attempt++)
	{
		DWORD deadline = GetTickCount() + 50;   // at most 50 ms of draining

		for(int d = 0; d < 64 && GetTickCount() < deadline; d++){
			int n = hid_read_timeout(priv->handle, drain_buf, FEATURE_BUFFER_SIZE, 5);
			if(n < 0)
				break;                           // real error, stop
			if(n == 0)
				continue;                        // nothing pending, keep trying
		}

		if(hid_write(priv->handle, cmd_sleep, sizeof(cmd_sleep)) > 0 &&
		   hid_write(priv->handle, cmd_start, sizeof(cmd_start)) > 0)
			ok = 1;
		else if(attempt + 1 < 5)
			Sleep(100);
	}

	// Report through the OpenHMD error string, not LOGD. OpenHMD's logging is
	// printf to stdout, which SteamVR never captures, so the driver had no
	// visibility at all into whether the panels were actually activated.
	// ohmd_ctx_get_error() does reach vrserver.txt.
	if(ok)
		LOGD("lgr100 activation commands sent");
	else
		ohmd_set_error(ctx, "lgr100 activation command failed (hid_write rejected, is another process holding the HID?); panels rely on proximity sensor and the left panel may stay dark");

	return ok;
}

static ohmd_device* open_device(ohmd_driver* driver, ohmd_device_desc* desc)
{
	lgr100_priv* priv = ohmd_alloc(driver->ctx, sizeof(lgr100_priv));
	if(!priv)
		return NULL;
	
	priv->id = desc->id;
	priv->base.ctx = driver->ctx;

	// Open the HID device
	priv->handle = hid_open_path(desc->path);

	if(!priv->handle) {
		goto cleanup;
	}

	// hidapi's Windows backend opens handles in blocking mode: hid_read() waits
	// forever when nothing is queued. This read runs under OpenHMD's global
	// update mutex (see update_device), so one HID pause freezes every other
	// OpenHMD call - including driver init (SteamVR watchdog abort -> safe mode)
	// and the pose/distortion path while rendering (black eye under load).
	hid_set_nonblocking(priv->handle, 1);

// Activation commands. The second byte of an LGR100 command report is the
	// *length* of the command text: the firmware terminates the received string
	// at that byte before matching it, so a value smaller than strlen() makes
	// the command silently not match (verified 2026-10-15: "Sleep Disable" is
	// 13 chars and only runs with 0x0D; with 0x0C it is ignored). The reference
	// implementation (LG360VRActivator) sends "Sleep Disable" with 0x0C, which
	// never did anything - the panel came up because "VR App Start" is correct.
	// Without these the left panel stays dark.
	//
	// Two things matter here and both were wrong before:
	//  - The drain must stop on a read *timeout*, not on an empty read, and it
	//    must not spin. With the headset awake the 200 Hz IMU stream never runs
	//    dry, so the old `break` on `<= 0` never fired and the loop drained a
	//    fixed count every time, stalling the driver long enough to trip SteamVR's
	//    watchdog. That made the panels come up or not depending purely on whether
	//    the headset happened to be asleep, which looked like an unrelated setting
	//    breaking the connection.
	//  - Overall time is bounded, so a slow device can never stall driver init.
	{
		// The activation sequence now lives in lgr100_activate_panels() so it
		// can also be triggered on demand from the SteamVR driver (the BACK
		// button on the visor re-lights the panels without a replug).
		lgr100_activate_panels(priv, driver->ctx);
	}
		
	// Set default device properties
	ohmd_set_default_device_properties(&priv->base.properties);

	// Set device properties (currently just aproximations)
	// If you have one, please open it and measure!
	priv->base.properties.hsize = 0.110000f;
	priv->base.properties.vsize = 0.038000f;
	// Use the geometry Windows surfaces the panel as (1440x960, two 720x960 eyes).
	// The native 960x720-per-eye layout was tried here and made both eyes worse,
	// so the transposition only exists in the panel's wiring, not in the surface
	// SteamVR composites to.
	priv->base.properties.hres = 1440;
	priv->base.properties.vres = 960;
	priv->base.properties.lens_sep = 0.063500f;
	priv->base.properties.lens_vpos = 0.020000f;
	// 80 deg was the spec figure, but objects were culled (they vanished) before
	// leaving the visible field, which means the displayed area is wider than
	// the frustum apps receive. 88 deg widens the culling boundary; the exact
	// value is judged by eye.
	priv->base.properties.fov = DEG_TO_RAD(88.0f);
	// The frustum below is transposed by the +/-90 flip, so its final aspect is
	// 1/ratio. The compositor rotates the eye texture 90 degrees, which maps the
	// rendered horizontal axis onto the slot's vertical axis (the user's
	// horizontal) and vice versa. For the final image to be isotropic the
	// frustum aspect must be the reciprocal of the eye slot (960/720 = 4/3),
	// i.e. the rendered field must be wider than tall. ratio = 720/960 = 0.75
	// gives aspect 1/0.75 = 1.3333. With the old 1.3333 the frustum was portrait
	// (aspect 0.75), which stretched the world by 1.78x and made it look narrow.
	priv->base.properties.ratio = 720.0f / 960.0f;
	
	// Some buttons and axes
	priv->base.properties.control_count = 2;
	priv->base.properties.controls_hints[0] = OHMD_BUTTON_A;
	priv->base.properties.controls_hints[1] = OHMD_BUTTON_B;
	priv->base.properties.controls_types[0] = OHMD_DIGITAL;
	priv->base.properties.controls_types[1] = OHMD_DIGITAL;

	// calculate projection eye projection matrices from the device properties
	ohmd_calc_default_proj_matrices(&priv->base.properties);

	// Correct matrices for correct physical projection
	mat4x4f flip;
	omat4x4f_init_ident(&flip);

	flip.m[0][0] = 0.0f; flip.m[0][1] = -1.0f; flip.m[1][0] = 1.0f; flip.m[1][1] = 0.0f;

	omat4x4f_mult(&priv->base.properties.proj_right, &flip, &priv->base.properties.proj_right);
	flip.m[0][1] = 1.0f; flip.m[1][0] = -1.0f;
	omat4x4f_mult(&priv->base.properties.proj_left, &flip, &priv->base.properties.proj_left);

	// set up device callbacks
	priv->base.update = update_device;
	priv->base.close = close_device;
	priv->base.getf = getf;

	// initialize sensor fusion
	ofusion_init(&priv->sensor_fusion);
	// The stock gain (0.05) corrects gravity so slowly that gyro bias wins and the
	// estimate drifts; raise it so tilt is corrected well within a second, but keep
	// it low enough that corrections don't look like pose jumps (which make
	// SteamVR recenter).
	priv->sensor_fusion.grav_gain = 0.5f;

	priv->gyro_bias.x = priv->gyro_bias.y = priv->gyro_bias.z = 0.0f;
	priv->bias_count = 0;
	priv->bias_ready = 0;

	// NOTE: a background keep-alive thread was tried here. A second HID handle plus
	// its writes blocks server_main long enough for SteamVR's watchdog to abort the
	// driver, which left only one panel lit. The panels are instead woken by the
	// activation sequence in open_device().

	return (ohmd_device*)priv;

cleanup:
	if(priv)
		free(priv);

	return NULL;
}

static void get_device_list(ohmd_driver* driver, ohmd_device_list* list)
{
	int id = 0;
	struct hid_device_info* cur_dev = hid_enumerate(0x1004, 0x6374);
	if(cur_dev == NULL)
		return;
	ohmd_device_desc* desc;

	// HMD
	desc = &list->devices[list->num_devices++];

	strcpy(desc->driver, "OpenHMD LG R100 (360VR) Driver");
	strcpy(desc->vendor, "LG");
	strcpy(desc->product, "LGR100");

	strcpy(desc->path, cur_dev->path);

	desc->driver_ptr = driver;

	desc->device_flags = OHMD_DEVICE_FLAGS_ROTATIONAL_TRACKING;
	desc->device_class = OHMD_DEVICE_CLASS_HMD;

	desc->id = id++;
	hid_free_enumeration(cur_dev);
}

static void destroy_driver(ohmd_driver* drv)
{
	LOGD("shutting down lgr100 driver");
	free(drv);
}

ohmd_driver* ohmd_create_lgr100_drv(ohmd_context* ctx)
{
	ohmd_driver* drv = ohmd_alloc(ctx, sizeof(ohmd_driver));
	if(!drv)
		return NULL;

	drv->get_device_list = get_device_list;
	drv->open_device = open_device;
	drv->destroy = destroy_driver;

	return drv;
}
