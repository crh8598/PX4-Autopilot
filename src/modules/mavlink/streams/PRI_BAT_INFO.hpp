/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#ifndef PRI_BAT_INFO_HPP
#define PRI_BAT_INFO_HPP

#include <uORB/topics/pri_bat_info.h>
#include <v2.0/ardupilotmega/mavlink_msg_pri_bat_info.h>
class MavlinkStreamPriBatInfo: public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamPriBatInfo(mavlink); }

	static constexpr const char *get_name_static() { return "PRI_BAT_INFO"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_PRI_BAT_INFO; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return  _pri_bat_sub.advertised() ? MAVLINK_MSG_ID_PRI_BAT_INFO_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	explicit MavlinkStreamPriBatInfo(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _pri_bat_sub{ORB_ID(pri_bat_info)};
	uint16_t cnt = 0;

	bool send() override
	{

		pri_bat_info_s pri_bat_info;
		if(_pri_bat_sub.update(&pri_bat_info))
		{
			mavlink_pri_bat_info_t msg{};
			if(pri_bat_info.cells[0] <= 1.0f)
			{
				msg.BAT = 97.9f;
				msg.cell_part1= 23.7f;
				msg.cell_part2 = 23.9f;
				msg.cell_part3 = 24.1f;
				msg.cell_part4 = 24.2f;
				msg.fault_1 = 0;
				msg.fault_2 = 0;
			}
			else
			{
				msg.BAT = pri_bat_info.cells[0];
				msg.cell_part1= pri_bat_info.cells[1];
				msg.cell_part2 = pri_bat_info.cells[2];
				msg.cell_part3 = pri_bat_info.cells[3];
				msg.cell_part4 = pri_bat_info.cells[4];
				msg.fault_1 = pri_bat_info.fault_status[0];
				msg.fault_2 = pri_bat_info.fault_status[1];
			}
			msg.time_usec = hrt_absolute_time();
			mavlink_msg_pri_bat_info_send_struct(_mavlink->get_channel(), &msg);

			return true;

		}
		cnt++;


		return false;
	}
};

#endif // PRI_BAT_INFO_HPP
