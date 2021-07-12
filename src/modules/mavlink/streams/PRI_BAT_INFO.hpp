
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

	const char *get_name() const override
	{
		return MavlinkStreamPriBatInfo::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		 return "PRI_BAT_INFO";
	}

	static constexpr uint16_t get_id_static()
	{
		 return MAVLINK_MSG_ID_PRI_BAT_INFO;
	}

	uint16_t get_id() override
	{
		 return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		PX4_INFO("pri_bat mavlink message created!");
		return new MavlinkStreamPriBatInfo(mavlink);
	}

	unsigned get_size() override
	{
		return  _pri_bat_sub.advertised() ? MAVLINK_MSG_ID_PRI_BAT_INFO_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}


private:
	uORB::Subscription _pri_bat_sub{ORB_ID(pri_bat_info)};
	uint64_t _pri_bat_info_time;

	// do not allow top copying this class
	MavlinkStreamPriBatInfo(MavlinkStreamPriBatInfo &) = delete;
	MavlinkStreamPriBatInfo &operator = (const MavlinkStreamPriBatInfo &) = delete;

protected:
	explicit MavlinkStreamPriBatInfo(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}
	bool send() override
	{

		pri_bat_info_s pri_bat_info{};
		while(_pri_bat_sub.update(&pri_bat_info))
		{
			mavlink_pri_bat_info_t msg = {};
			_pri_bat_sub.copy(&pri_bat_info);

			msg.bat = pri_bat_info.bat;
			msg.cell_part1= pri_bat_info.cell_part1;
			msg.cell_part2 = pri_bat_info.cell_part2;
			msg.cell_part3 = pri_bat_info.cell_part3;
			msg.cell_part4 = pri_bat_info.cell_part4;
			msg.fault_1 = pri_bat_info.fault_1;
			msg.fault_2 = pri_bat_info.fault_2;
			msg.time_usec = pri_bat_info.timestamp;

			mavlink_msg_pri_bat_info_send_struct(_mavlink->get_channel(), &msg);
// mavlink_msg_pri_bat_info_send(_mavlink->get_channel(), msg.time_usec, msg.bat, msg.cell_part1, msg.cell_part2, msg.cell_part3, msg.cell_part4, msg.fault_1, msg.fault_2);
			return true;

		}

		return false;
	}
};

#endif // PRI_BAT_INFO_HPP
