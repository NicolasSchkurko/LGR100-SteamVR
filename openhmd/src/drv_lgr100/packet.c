// Copyright 2018, Joey Ferwerda.
// SPDX-License-Identifier: BSL-1.0
/*
 * OpenHMD - Free and Open Source API and drivers for immersive technology.
 */

/* LG R100 (360VR) Driver */

#include "lgr100.h"

#define READFLOAT (*(float*)buffer); buffer += 4;

bool decode_lgr100_imu_msg(lgr100_sensor_sample* smp, const unsigned char* buffer, int size)
{
	// On Windows (hidapi-winapi) reports come padded to the max packet size (64),
	// while on Linux they come as 31/32 bytes. Accept anything >= the 30-byte body.
	if(size < 30){
		LOGE("invalid packet size (expected >= 30 but got %d)", size);
		return false;
	}

	buffer += 1;
	
	for(int i = 0; i < 3; i++){
		smp->gyro[i] = READFLOAT;
	}

	for(int i = 0; i < 3; i++){
		smp->accel[i] = READFLOAT;
	}

	// Sample tick/counter (observed at bytes 26..29 on Windows 64-byte reports)
	memcpy(&smp->tick, buffer + 1, 4);
	return true;
}