/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file px4_mavlink_debug.cpp
 * Debug application example for PX4 autopilot
 *
 * @author Example User <mail@example.com>
 */

#include <px4_platform_common/px4_config.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <poll.h>

#include <systemlib/err.h>
#include <drivers/drv_hrt.h>

#include <uORB/uORB.h>
#include <uORB/topics/debug_key_value.h>
#include <uORB/topics/debug_value.h>
#include <uORB/topics/debug_vect.h>
#include <uORB/topics/debug_array.h>
#include <uORB/topics/pri_bat_info.h>


extern "C" __EXPORT int px4_mavlink_debug_main(int argc, char *argv[]);

int px4_mavlink_debug_main(int argc, char *argv[])
{
	printf("Hello Debug!\n");

	/* advertise named debug value */
	//uorb의 메세지는 이름뒤에 _s 를 붙혀 구조체 형으로 사용한다. 메세지의 위치 : msg/debug_key_value.msg
	struct debug_key_value_s dbg_key;
	// 10 char copy to dbg_key.key
	strncpy(dbg_key.key, "velx", 10);
	dbg_key.value = 0.0f;
	// make new Handler for debug_key_value and insert data which formed by structure
	orb_advert_t pub_dbg_key = orb_advertise(ORB_ID(debug_key_value), &dbg_key);

	/* advertise indexed debug value */
	struct debug_value_s dbg_ind;
	dbg_ind.ind = 42;
	dbg_ind.value = 0.5f;
	orb_advert_t pub_dbg_ind = orb_advertise(ORB_ID(debug_value), &dbg_ind);

	/* advertise debug vect */
	struct debug_vect_s dbg_vect;
	strncpy(dbg_vect.name, "vel3D", 10);
	dbg_vect.x = 1.0f;
	dbg_vect.y = 2.0f;
	dbg_vect.z = 3.0f;
	orb_advert_t pub_dbg_vect = orb_advertise(ORB_ID(debug_vect), &dbg_vect);

	/* advertise debug array */
	struct debug_array_s dbg_array;
	dbg_array.id = 1;
	strncpy(dbg_array.name, "dbg_array", 10);
	orb_advert_t pub_dbg_array = orb_advertise(ORB_ID(debug_array), &dbg_array);

	/* ASTROX BMS */
	struct pri_bat_info_s pbi;
	pbi.bat = 96.5;
	pbi.cell_part1=23.3;
	pbi.cell_part2=24.1;
	pbi.cell_part3=24.4;
	pbi.cell_part4=24.7;
	pbi.fault_1 = 0;
	pbi.fault_2 = 0;
	orb_advert_t pub_pri_bat_info = orb_advertise(ORB_ID(pri_bat_info),&pbi);

	int value_counter = 0;

	while (value_counter < 1000) {
		uint64_t timestamp_us = hrt_absolute_time();

		/* send one named value */
		dbg_key.value = value_counter;
		dbg_key.timestamp = timestamp_us;
		orb_publish(ORB_ID(debug_key_value), pub_dbg_key, &dbg_key);

		/* send one indexed value */
		dbg_ind.value = 0.5f * value_counter;
		dbg_ind.timestamp = timestamp_us;
		orb_publish(ORB_ID(debug_value), pub_dbg_ind, &dbg_ind);

		/* send one vector */
		dbg_vect.x = 1.0f * value_counter;
		dbg_vect.y = 2.0f * value_counter;
		dbg_vect.z = 3.0f * value_counter;
		dbg_vect.timestamp = timestamp_us;
		orb_publish(ORB_ID(debug_vect), pub_dbg_vect, &dbg_vect);

		/* ASTROX, Send pri bat info */
		pbi.timestamp = timestamp_us;
		orb_publish(ORB_ID(pri_bat_info),pub_pri_bat_info,&pbi);

		/* send one array */
		for (size_t i = 0; i < debug_array_s::ARRAY_SIZE; i++) {
			dbg_array.data[i] = value_counter + i * 0.01f;
		}

		dbg_array.timestamp = timestamp_us;
		orb_publish(ORB_ID(debug_array), pub_dbg_array, &dbg_array);

		warnx("sent one more value..");

		value_counter++;
		px4_usleep(500000);
	}

	return 0;
}
