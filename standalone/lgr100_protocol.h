// Wire protocol of the LG 360 VR (R100) headset.
//
// Extracted from the vendored OpenHMD LGR100 driver
// (openhmd/src/drv_lgr100/lgr100.h, Joey Ferwerda, BSL-1.0) so the SteamVR
// driver can talk to the headset directly, without OpenHMD in between.

#ifndef LGR100_PROTOCOL_H
#define LGR100_PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FEATURE_BUFFER_SIZE 256

// USB vendor / product id of the headset's HID interface.
#define LGR100_VID 0x1004
#define LGR100_PID 0x6374

typedef enum
{
	LGR100_BUTTON_OK_ON    = 1,
	LGR100_BUTTON_BACK_ON  = 2,
	LGR100_BUTTON_BACK_OFF = 3,
	LGR100_BUTTON_OK_OFF   = 4,
} lgr100_button;

typedef struct
{
	float accel[3];
	float gyro[3];
	uint32_t tick;
} lgr100_sensor_sample;

// Report ids of the HID interface (5 report ids, 63 bytes in/out).
typedef enum {
	LGR100_IRQ_NULL			= 0,
	LGR100_IRQ_PROXIMITY	= 1,
	LGR100_IRQ_BUTTONS		= 2,
	LGR100_IRQ_DEBUG1		= 3,
	LGR100_IRQ_DEBUG2		= 4,
	LGR100_IRQ_SENSORS 		= 5,
	LGR100_IRQ_DEBUG_SEQ1 	= 32,
	LGR100_IRQ_DEBUG_SEQ2 	= 33,
	LGR100_IRQ_UNKNOWN3		= 101,
	LGR100_IRQ_UNKNOWN2		= 255
} lgr100_usb_cmd;

// Commands. The second byte of a command report is the *length* of the
// command text: the firmware terminates the received string at that byte
// before matching it, so a value smaller than strlen() makes the command
// silently not match (verified 2026-10-15: "Sleep Disable" is 13 chars and
// only runs with 0x0D; with 0x0C it is ignored).
//
// "Sleep Disable" keeps the device awake (it disables the sleep/proximity
// power down behaviour and wakes the display path). "VR App Start" starts VR
// app mode: it sets the is_vr_app_start flag and turns the panel backlight
// on. Without them the panels stay dark.
static const unsigned char lgr100_cmd_sleep_disable[15] =
	{0x03, 0x0D, 'S','l','e','e','p',' ','D','i','s','a','b','l','e'};
static const unsigned char lgr100_cmd_vr_app_start[14] =
	{0x03, 0x0C, 'V','R',' ','A','p','p',' ','S','t','a','r','t'};

bool decode_lgr100_imu_msg(lgr100_sensor_sample* smp, const unsigned char* buffer, int size);

#ifdef __cplusplus
}
#endif

#endif
