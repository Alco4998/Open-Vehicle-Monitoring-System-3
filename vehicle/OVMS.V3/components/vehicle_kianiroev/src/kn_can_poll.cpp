/*
;    Project:       Open Vehicle Monitor System
;    Date:          21th January 2019
;
;    (C) 2019       Geir Øyvind Vælidalo <geir@validalo.net>
;
; Permission is hereby granted, free of charge, to any person obtaining a copy
; of this software and associated documentation files (the "Software"), to deal
; in the Software without restriction, including without limitation the rights
; to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
; copies of the Software, and to permit persons to whom the Software is
; furnished to do so, subject to the following conditions:
;
; The above copyright notice and this permission notice shall be included in
; all copies or substantial portions of the Software.
;
; THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
; IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
; FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
; AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
; LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
; OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
; THE SOFTWARE.
*/
#include "vehicle_kianiroev.h"

static const char *TAG = "v-kianiroev";

/**
 * Incoming poll reply messages
 */
void OvmsVehicleKiaNiroEv::IncomingPollReply(const OvmsPoller::poll_job_t& job, uint8_t* data, uint8_t length)
{
	/* ESP_LOGD(TAG, "IPR %03x TYPE:%x PID:%02x %x %02x %02x %02x %02x %02x %02x %02x %02x", job.moduleid_low, job.type, job.pid, length, data[0], data[1], data[2], data[3],
		data[4], data[5], data[6], data[7]); */
	bool process_all = false;
	switch (job.moduleid_rec)
	{
		// ****** IGMP *****
	case 0x778:
		IncomingIGMP(job.bus, job.type, job.pid, data, length, job.mlframe, job.mlremain);
		break;

		// ****** OBC ******
	case 0x7ed:
		IncomingOBC(job.bus, job.type, job.pid, data, length, job.mlframe, job.mlremain);
		break;

		// ****** BCM ******
	case 0x7a8:
		IncomingBCM(job.bus, job.type, job.pid, data, length, job.mlframe, job.mlremain);
		break;

		// ****** AirCon ******
	case 0x7bb:
		process_all = true;
		// IncomingAirCon(job.bus, job.type, job.pid, data, length, job.mlframe, job.mlremain);
		break;

		// ****** ABS ESP ******
	case 0x7d9:
		IncomingAbsEsp(job.bus, job.type, job.pid, data, length, job.mlframe, job.mlremain);
		break;

		// ******* VMCU ******
	case 0x7ea:
		IncomingVMCU(job.bus, job.type, job.pid, data, length, job.mlframe, job.mlremain);
		break;

		// ******* MCU ******
	case 0x7eb:
		IncomingMCU(job.bus, job.type, job.pid, data, length, job.mlframe, job.mlremain);
		break;

		// ***** BMC ****
	case 0x7ec:
		IncomingBMC(job.bus, job.type, job.pid, data, length, job.mlframe, job.mlremain);
		break;

		// ***** CM ****
	case 0x7ce:
		IncomingCM(job.bus, job.type, job.pid, data, length, job.mlframe, job.mlremain);
		break;

	default:
		ESP_LOGD(TAG, "Unknown module: %03" PRIx32, job.moduleid_rec);
		break;
	}

	// Enabled above as they are converted.
	if (process_all)
	{
		std::string& rxbuf = obd_rxbuf;

		// Assemble first and following frames to get complete reply

		// init rx buffer on first (it tells us whole length)
		if (job.mlframe == 0)
		{
			obd_module = job.moduleid_rec;
			obd_rxtype = job.type;
			obd_rxpid = job.pid;
			obd_frame = 0;
			rxbuf.clear();
			ESP_LOGV(TAG, "IoniqISOTP: IPR %03" PRIx32 " TYPE:%x PID: %03x Buffer: %d - Start",
					 job.moduleid_rec, job.type, job.pid, length + job.mlremain);
			rxbuf.reserve(length + job.mlremain);
		}
		else
		{
			if (obd_frame == 0xffff)
			{
				return; // Aborted
			}
			if ((obd_rxtype != job.type) || (obd_rxpid != job.pid) || (obd_module != job.moduleid_rec))
			{
				ESP_LOGD(TAG, "IoniqISOTP: IPR %03" PRIx32 " TYPE:%x PID: %03x Dropped Frame",
						 job.moduleid_rec, job.type, job.pid);
				return;
			}
			++obd_frame;
			if (obd_frame != job.mlframe)
			{
				obd_frame = 0xffff;
				rxbuf.clear();
				ESP_LOGD(TAG, "IoniqISOTP: IPR %03" PRIx32 " TYPE:%x PID: %03x Skipped Frame: %d",
						 job.moduleid_rec, job.type, job.pid, obd_frame);
				return;
			}
		}
		// Append each piece..
		rxbuf.insert(rxbuf.end(), data, data + length);
		/*
		ESP_LOGV(TAG, "IoniqISOTP: IPR %03x TYPE:%x PID: %03x Frame: %d Append: %d Expected: %d - Append",
		  job.moduleid_rec, job.type, job.pid, m_poll_ml_frame, length, job.mlremain);
		*/
		if (job.mlremain > 0)
		{
			// we need more - return for now.
			return;
		}

		Incoming_Full(job.type, job.moduleid_sent, job.moduleid_rec, job.pid, rxbuf);
		obd_frame = 0xffff; // Received all - drop until we have a new frame 0
		rxbuf.clear();
	}
}

void OvmsVehicleKiaNiroEv::Incoming_Full(uint16_t type, uint32_t module_sent, uint32_t module_rec, uint16_t pid, const std::string &data)
{
	ESP_LOGD(TAG, "IoniqISOTP: IPR %03" PRIx32 " TYPE:%x PID: %03x Message Size: %d",
			 module_rec, type, pid, data.size());
	ESP_BUFFER_LOGV(TAG, data.data(), data.size());

	switch (type)
	{
	case VEHICLE_POLL_TYPE_READDATA:
		switch (module_rec)
		{
		case 0x7bb:
			IncomingFull_AirCon(type, pid, data);
			break;

		default:
			ESP_LOGD(TAG, "Unkown module_rec");
			break;
		}
		break;

	default:
		ESP_LOGD(TAG, "Unkown type");
		break;
	}
}

/**
 * Handle incoming messages from cluster.
 */
void OvmsVehicleKiaNiroEv::IncomingCM(canbus *bus, uint16_t type, uint16_t pid, const uint8_t *data, uint8_t length, uint16_t mlframe, uint16_t mlremain)
{
	//	ESP_LOGI(TAG, "CM PID:%02x %x %02x %02x %02x %02x %02x %02x %02x %02x", pid, length, mlframe, data[0], data[1], data[2], data[3],
	//			data[4], data[5], data[6]);
	//	ESP_LOGI(TAG, "---");
	switch (pid)
	{
	case 0xb002:
		if (IsKona())
		{
			if (mlframe == 1)
			{
				odo = CAN_BYTE(6) << 16;
			}
			if (mlframe == 2)
			{
				odo += CAN_UINT(0);
				StdMetrics.ms_v_pos_odometer->SetValue(odo, GetConsoleUnits());
			}
		}
		else
		{
			if (mlframe == 1)
			{
				StdMetrics.ms_v_pos_odometer->SetValue(CAN_UINT24(3), GetConsoleUnits());
			}
		}
		break;
	}
}

/**
 * Handle incoming messages from Aircon poll.
 */
void OvmsVehicleKiaNiroEv::IncomingAirCon(canbus *bus, uint16_t type, uint16_t pid, const uint8_t *data, uint8_t length, uint16_t mlframe, uint16_t mlremain)
{
	/* ESP_LOGD(TAG, "AirCon PID:%02x %x %02x %02x %02x %02x %02x %02x %02x %02x", pid, length, mlframe, data[0], data[1], data[2], data[3],
			data[4], data[5], data[6]); */
	switch (pid)
	{
	case 0x0100:
		if (mlframe == 1)
		{
			StdMetrics.ms_v_env_cabintemp->SetValue((data[2] / 2.0) - 40, Celcius);
			StdMetrics.ms_v_env_temp->SetValue((data[3] / 2.0) - 40, Celcius);
		}
		if (mlframe == 4)
		{
			StdMetrics.ms_v_pos_speed->SetValue(CAN_BYTE(5)); // Speed moved to here --TP
		}
		break;

	case 0x0102:
		if (mlframe == 1)
		{
			// Coolant temp 1 & 2 in byte 1 and 2
		}
		break;
	}
}

void OvmsVehicleKiaNiroEv::IncomingFull_AirCon(uint16_t type, uint16_t pid, const std::string &data)
{
	switch (pid)
	{
		case 0x0100:
		{
			uint8_t value;
			if (get_uint_buff_be<1>(data, 2, value))
			{
				StdMetrics.ms_v_env_cabintemp->SetValue((value / 2.0) - 40, Celcius);
			}
			if (get_uint_buff_be<1>(data, 3, value))
			{
				StdMetrics.ms_v_env_temp->SetValue((value / 2.0) - 40, Celcius);
			}
			if (get_uint_buff_be<1>(data, 29, value))
			{
				StdMetrics.ms_v_pos_speed->SetValue(value);
				CalculateAcceleration();
			}
		}
		break;
	}
}

/**
 * Handle incoming messages from ABS ESP poll.
 */
void OvmsVehicleKiaNiroEv::IncomingAbsEsp(canbus *bus, uint16_t type, uint16_t pid, const uint8_t *data, uint8_t length, uint16_t mlframe, uint16_t mlremain)
{
	// ESP_LOGD(TAG, "ABS/ESP PID:%02x %x %02x %02x %02x %02x %02x %02x %02x %02x", pid, length, mlframe, data[0], data[1], data[2], data[3],
	//		data[4], data[5], data[6]);
	switch (pid)
	{
	case 0xC101:
		if (mlframe == 3)
		{
			//				m_v_emergency_lights->SetValue((CAN_BYTE(2)>>6) & 1);
			m_v_emergency_lights->SetValue(CAN_BIT(2, 6));
			m_v_traction_control->SetValue(CAN_BIT(1, 6));
		}
		break;
	}
}

/**
 * Handle incoming messages from On Board Charger-poll
 *
 * - OBC-voltage
 * - Pilot signal duty cycle
 * - Charger temperature
 */
void OvmsVehicleKiaNiroEv::IncomingOBC(canbus *bus, uint16_t type, uint16_t pid, const uint8_t *data, uint8_t length, uint16_t mlframe, uint16_t mlremain)
{
	switch (pid)
	{
	case 0x01:
		if (mlframe == 4)
		{
			m_obc_pilot_duty->SetValue((float)CAN_BYTE(3) / 10.0); // Untested
		}
		else if (mlframe == 6)
		{
			StdMetrics.ms_v_charge_temp->SetValue((float)(CAN_BYTE(3) / 2.0) - 40.0, Celcius); // Untested
			kia_obc_ac_voltage = (float)CAN_BYTE(6);
		}
		else if (mlframe == 7)
		{
			float main_batt_voltage = CAN_UINT(3) / 10.0;
			ESP_LOGD(TAG, "OBC Main batt: %f", main_batt_voltage);
		}
		break;

	case 0x03:
		if (mlframe == 1)
		{
			kia_obc_ac_current = (float)CAN_UINT(0) / 100.0;
		}
		break;
	}
}

/**
 * Handle incoming messages from VMCU-poll
 *
 * - Aux battery SOC, Voltage and current
 */
void OvmsVehicleKiaNiroEv::IncomingVMCU(canbus *bus, uint16_t type, uint16_t pid, const uint8_t *data, uint8_t length, uint16_t mlframe, uint16_t mlremain)
{
	// ESP_LOGD(TAG, "VMCU TYPE: %02x PID:%02x %x %02x %02x %02x %02x %02x %02x %02x %02x", type, pid, length, mlframe, data[0], data[1], data[2], data[3],
	//		data[4], data[5], data[6]);

	switch (pid)
	{
	case 0x01:
		if (type == VEHICLE_POLL_TYPE_OBDIIGROUP)
		{
			if (mlframe == 1)
			{
				uint8_t shift_status = data[1] & 0xF;
				kn_shift_bits.Park = shift_status == 1;
				kn_shift_bits.Reverse = shift_status == 2;
				kn_shift_bits.Neutral = shift_status == 4;
				kn_shift_bits.Drive = shift_status == 8;

				// ESP_LOGD(TAG, "VMCU PID:%02x %x %02x %02x %02x %02x %02x %02x %02x %02x", pid, length, mlframe, data[0], data[1], data[2], data[3],
				//		data[4], data[5], data[6]);

				// ESP_LOGD(TAG, "ABS/ESP %02x %02x", data[1], data[2]);

				if (kn_shift_bits.Reverse)
				{
					StdMetrics.ms_v_env_gear->SetValue(-1);
				}
				else if (kn_shift_bits.Drive)
				{
					StdMetrics.ms_v_env_gear->SetValue(1);
				}
				else
				{
					StdMetrics.ms_v_env_gear->SetValue(0);
				}
			}
			else if (mlframe == 2)
			{
				// StdMetrics.ms_v_pos_speed->SetValue(CAN_UINT(2)/100.0);
				// StdMetrics.ms_v_pos_speed->SetValue(CAN_UINT(1)/10.0); // Alex said the other one was wrong. Maybe this is correct? No speed is in 7b3 220100 frame 4 Byte 5
			}
		}
		break;

	case 0x02:
		if (type == VEHICLE_POLL_TYPE_OBDIIGROUP)
		{
			if (mlframe == 3)
			{
				StdMetrics.ms_v_bat_12v_voltage->SetValue(((CAN_BYTE(2) << 8) + CAN_BYTE(1)) / 1000.0, Volts);
				// ms_v_bat_12v_current doesn't seem to be right
				StdMetrics.ms_v_bat_12v_current->SetValue((((int16_t)CAN_BYTE(4) << 8) + CAN_BYTE(3)) / 1000.0, Amps);
				m_b_aux_soc->SetValue(CAN_BYTE(5), Percentage);
			}
		}
		break;

	case 0x80:
		if (type == VEHICLE_POLL_TYPE_OBDII_1A)
		{
			if (mlframe == 2)
			{
				m_vin[0] = CAN_BYTE(3);
				m_vin[1] = CAN_BYTE(4);
				m_vin[2] = CAN_BYTE(5);
				m_vin[3] = CAN_BYTE(6);
			}
			else if (mlframe == 3)
			{
				m_vin[4] = CAN_BYTE(0);
				m_vin[5] = CAN_BYTE(1);
				m_vin[6] = CAN_BYTE(2);
				m_vin[7] = CAN_BYTE(3);
				m_vin[8] = CAN_BYTE(4);
				m_vin[9] = CAN_BYTE(5);
				m_vin[10] = CAN_BYTE(6);
			}
			else if (mlframe == 4)
			{
				m_vin[11] = CAN_BYTE(0);
				m_vin[12] = CAN_BYTE(1);
				m_vin[13] = CAN_BYTE(2);
				m_vin[14] = CAN_BYTE(3);
				m_vin[15] = CAN_BYTE(4);
				m_vin[16] = CAN_BYTE(5);
				StandardMetrics.ms_v_vin->SetValue(m_vin);
			}
		}
		break;
	}
}

/**
 * Handle incoming messages from VMCU-poll
 *
 * -
 */
void OvmsVehicleKiaNiroEv::IncomingMCU(canbus *bus, uint16_t type, uint16_t pid, const uint8_t *data, uint8_t length, uint16_t mlframe, uint16_t mlremain)
{
	//	ESP_LOGI(TAG, "MCU PID:%02x %x %02x %02x %02x %02x %02x %02x %02x %02x", pid, length, mlframe, data[0], data[1], data[2], data[3],
	//				data[4], data[5], data[6]);
	//	ESP_LOGI(TAG, "-");
	switch (pid)
	{
	case 0x02:
		if (type == VEHICLE_POLL_TYPE_OBDIIGROUP)
		{
			if (mlframe == 2)
			{
				StdMetrics.ms_v_mot_temp->SetValue((int8_t)CAN_BYTE(4)); // TODO Correct? Could be byte 2 *2
				StdMetrics.ms_v_inv_temp->SetValue((int8_t)CAN_BYTE(3)); // TODO Correct? Could be byte 1 *2
			}
			//				else if (mlframe == 3)
			//					{
			//					ESP_LOGD(TAG, "VMCU PID:%02x %x %02x %02x %02x %02x %02x %02x %02x %02x", pid, length, mlframe, data[0], data[1], data[2], data[3],
			//							data[4], data[5], data[6]);
			//					}
		}
		break;
	}
}

/**
 * Handle incoming messages from BMC-poll
 *
 * - Pilot signal available
 * - CCS / Type2
 * - Battery current
 * - Battery voltage
 * - Battery module temp 1-8
 * - Cell voltage max / min + cell #
 * + more
 */
void OvmsVehicleKiaNiroEv::IncomingBMC(canbus *bus, uint16_t type, uint16_t pid, const uint8_t *data, uint8_t length, uint16_t mlframe, uint16_t mlremain)
{
	uint8_t bVal;
	if (type == VEHICLE_POLL_TYPE_OBDIIEXTENDED)
	{
		switch (pid)
		{
		case 0x0101:
			// diag page 01: skip first frame (no data)
			// ESP_LOGD(TAG, "Frame number %x",mlframe);
			if (mlframe == 1)
			{
				// ESP_LOGD(TAG, "BMC PID:%02x %x %02x %02x %02x %02x %02x %02x %02x %02x", pid, length, mlframe, data[0], data[1], data[2], data[3],
				// 		data[4], data[5], data[6]);

				m_b_bms_soc->SetValue(CAN_BYTE(1) / 2.0);
				// MAx regen: ((CAN_BYTE(1)<<8) + CAN_BYTE(2))/100
				m_c_power->SetValue((float)CAN_UINT(2) / 100.0, kW);
				// m_c_power->SetValue( (float)CAN_UINT(2)/100.0, kW);
				m_b_bms_relay->SetValue(CAN_BIT(6, 0));
			}
			else if (mlframe == 2)
			{
				StdMetrics.ms_v_bat_current->SetValue((float)CAN_INT(0) / 10.0, Amps);
				StdMetrics.ms_v_bat_voltage->SetValue((float)CAN_UINT(2) / 10.0, Volts);

				m_b_min_temperature->SetValue(CAN_BYTE(5));
				m_b_max_temperature->SetValue(CAN_BYTE(4));
				BmsRestartCellTemperatures();
				BmsSetCellTemperature(0, CAN_BYTE(6));
				StdMetrics.ms_v_bat_temp->SetValue((float)CAN_BYTE(5), Celcius); // TODO Should we use Min temp or Max Temp?
			}
			else if (mlframe == 3)
			{
				BmsSetCellTemperature(1, CAN_BYTE(0));
				BmsSetCellTemperature(2, CAN_BYTE(1));
				BmsSetCellTemperature(3, CAN_BYTE(2));
				m_b_inlet_temperature->SetValue(CAN_BYTE(5));
				m_b_cell_volt_max->SetValue((float)CAN_BYTE(6) / 50.0, Volts);
				StandardMetrics.ms_v_bat_cell_vmax->SetValue((float)CAN_BYTE(1) / 50.0, Volts);
			}
			else if (mlframe == 4)
			{
				m_b_cell_volt_max_no->SetValue(CAN_BYTE(0));
				m_b_cell_volt_min->SetValue((float)CAN_BYTE(1) / 50.0, Volts);
				StandardMetrics.ms_v_bat_cell_vmin->SetValue((float)CAN_BYTE(1) / 50.0, Volts);
				m_b_cell_volt_min_no->SetValue(CAN_BYTE(2));
				kn_battery_fan_feedback = CAN_BYTE(3);		  // TODO Battery fan feedback
				kn_charge_bits.FanStatus = CAN_BYTE(4) & 0xF; // TODO Battery fan speed
				kia_battery_cum_charge_current = (kia_battery_cum_charge_current & 0x00FFFFFF) | ((uint32_t)CAN_UINT(6) << 24);
			}
			else if (mlframe == 5)
			{
				kia_battery_cum_charge_current = (kia_battery_cum_charge_current & 0xFF000000) | ((uint32_t)CAN_UINT24(0));
				kia_battery_cum_discharge_current = CAN_UINT32(3);
			}
			else if (mlframe == 6)
			{
				kia_battery_cum_charge = CAN_UINT32(0);
				kia_battery_cum_discharge = (kia_battery_cum_discharge & 0xFF) | ((uint32_t)CAN_UINT24(4) << 8);
			}
			else if (mlframe == 7)
			{
				kia_battery_cum_discharge = (kia_battery_cum_discharge & 0xFFFFFF00) | ((uint32_t)CAN_BYTE(0));
				kia_battery_cum_op_time = CAN_UINT32(1) / 3600;
				m_b_bms_ignition->SetValue(CAN_BIT(5, 2));
			}
			break;

		case 0x0102: // Cell voltages
		case 0x0103:
		case 0x0104:
			// diag page 02-04: skip first frame (no data)
			if (mlframe == 0 && pid == 0x0102)
			{
				BmsRestartCellVoltages();
			}
			else if (mlframe > 0)
			{
				int8_t base = ((pid - 0x102) * 32) - 1 + (mlframe - 1) * 7;
				bVal = (mlframe == 1) ? 1 : 0;
				for (; bVal < length && ((base + bVal) < 96); bVal++)
				{
					BmsSetCellVoltage((uint8_t)(base + bVal), (float)CAN_BYTE(bVal) * 0.02);
				}

				/*
				Frame 1-5 contains Cell voltages
				1-6
				0-6
				0-6
				0-6
				0-4
				*/
			}
			break;
		case 0x0105:
			if (mlframe == 3)
			{
				m_b_heat_1_temperature->SetValue(CAN_BYTE(6));
			}
			else if (mlframe == 4)
			{
				StdMetrics.ms_v_bat_soh->SetValue((float)CAN_UINT(1) / 10.0);
				m_b_cell_det_max_no->SetValue(CAN_BYTE(3));
				m_b_cell_det_min->SetValue((float)CAN_UINT(4) / 10.0);
				m_b_cell_det_min_no->SetValue(CAN_BYTE(6));
			}
			else if (mlframe == 5)
			{
				StdMetrics.ms_v_bat_soc->SetValue(CAN_BYTE(0) / 2.0);
				BmsSetCellVoltage(96, (float)CAN_BYTE(3) * 0.02);
				BmsSetCellVoltage(97, (float)CAN_BYTE(4) * 0.02);
			}
			break;
		}
	}
}

/**
 * Handle incoming messages from BCM-poll
 *
 *
 */
void OvmsVehicleKiaNiroEv::IncomingBCM(canbus *bus, uint16_t type, uint16_t pid, const uint8_t *data, uint8_t length, uint16_t mlframe, uint16_t mlremain)
{
	uint8_t bVal;
	uint32_t lVal;

	if (type == VEHICLE_POLL_TYPE_OBDIIEXTENDED)
	{
		switch (pid)
		{
		case 0xB00E:
			if (mlframe == 1)
			{
				// Charge door port not yet found for Kona - we'll fake it for now
				if (!IsKona())
				{
					StdMetrics.ms_v_door_chargeport->SetValue(CAN_BIT(1, 4));
				}
			}
			break;

		case 0xB00C:
			if (mlframe == 1)
			{
				m_v_heated_handle->SetValue(CAN_BIT(1, 5));
			}
			break;

		case 0xC002:
			if (mlframe == 1)
			{
				SET_TPMS_ID(0, CAN_UINT32(1));
				lVal = (kia_tpms_id[1] & 0x0000ffff) | ((uint32_t)CAN_UINT(5) << 16);
				SET_TPMS_ID(1, lVal);
			}
			else if (mlframe == 2)
			{
				lVal = (uint32_t)CAN_UINT(0) | (kia_tpms_id[1] & 0xffff0000);
				SET_TPMS_ID(1, lVal);
				SET_TPMS_ID(2, CAN_UINT32(2));
				lVal = (kia_tpms_id[1] & 0x00ffffff) | ((uint32_t)CAN_BYTE(5) << 24);
				SET_TPMS_ID(3, lVal);
			}
			else if (mlframe == 3)
			{
				lVal = (uint32_t)CAN_UINT24(0) | (kia_tpms_id[3] & 0xff000000);
				SET_TPMS_ID(3, lVal);
			}
			break;

		case 0xC00B:
			if (mlframe == 1)
			{
				bVal = CAN_BYTE(1);
				if (bVal > 0)
					StdMetrics.ms_v_tpms_pressure->SetElemValue(MS_V_TPMS_IDX_FL, bVal / 5.0, PSI);
				bVal = CAN_BYTE(2);
				if (bVal > 0)
					StdMetrics.ms_v_tpms_temp->SetElemValue(MS_V_TPMS_IDX_FL, bVal - 50.0, Celcius);
				bVal = CAN_BYTE(5);
				if (bVal > 0)
					StdMetrics.ms_v_tpms_pressure->SetElemValue(MS_V_TPMS_IDX_FR, bVal / 5.0, PSI);
				bVal = CAN_BYTE(6);
				if (bVal > 0)
					StdMetrics.ms_v_tpms_temp->SetElemValue(MS_V_TPMS_IDX_FR, bVal - 50.0, Celcius);
			}
			else if (mlframe == 2)
			{
				bVal = CAN_BYTE(2);
				if (bVal > 0)
					StdMetrics.ms_v_tpms_pressure->SetElemValue(MS_V_TPMS_IDX_RR, bVal / 5.0, PSI);
				bVal = CAN_BYTE(3);
				if (bVal > 0)
					StdMetrics.ms_v_tpms_temp->SetElemValue(MS_V_TPMS_IDX_RR, bVal - 50.0, Celcius);
				bVal = CAN_BYTE(6);
				if (bVal > 0)
					StdMetrics.ms_v_tpms_pressure->SetElemValue(MS_V_TPMS_IDX_RL, bVal / 5.0, PSI);
			}
			else if (mlframe == 3)
			{
				bVal = CAN_BYTE(0);
				if (bVal > 0)
					StdMetrics.ms_v_tpms_temp->SetElemValue(MS_V_TPMS_IDX_RL, bVal - 50.0, Celcius);
			}
			break;
		}
	}
}

/**
 * Handle incoming messages from IGMP-poll
 *
 *
 */
void OvmsVehicleKiaNiroEv::IncomingIGMP(canbus *bus, uint16_t type, uint16_t pid, const uint8_t *data, uint8_t length, uint16_t mlframe, uint16_t mlremain)
{
	if (type == VEHICLE_POLL_TYPE_OBDIIEXTENDED)
	{
		switch (pid)
		{
		case 0xbc03:
			if (mlframe == 1)
			{
				StdMetrics.ms_v_door_hood->SetValue(CAN_BIT(2, 0));

				StdMetrics.ms_v_env_on->SetValue((CAN_BYTE(2) & 0x60) > 0);

				m_v_seat_belt_driver->SetValue(CAN_BIT(2, 1));
				m_v_seat_belt_passenger->SetValue(CAN_BIT(2, 2));

				/*
				 * Car data is by driver, but metrics is left/right
				 */
				if (IsLHD())
				{
					StdMetrics.ms_v_door_fl->SetValue(CAN_BIT(1, 5));
					StdMetrics.ms_v_door_fr->SetValue(CAN_BIT(1, 4));
					StdMetrics.ms_v_door_rl->SetValue(CAN_BIT(1, 0));
					StdMetrics.ms_v_door_rr->SetValue(CAN_BIT(1, 2));
					m_v_door_lock_rl->SetValue(CAN_BIT(1, 1));
					m_v_door_lock_rr->SetValue(CAN_BIT(1, 3));
				}
				else
				{
					StdMetrics.ms_v_door_fr->SetValue(CAN_BIT(1, 5));
					StdMetrics.ms_v_door_fl->SetValue(CAN_BIT(1, 4));
					StdMetrics.ms_v_door_rr->SetValue(CAN_BIT(1, 0));
					StdMetrics.ms_v_door_rl->SetValue(CAN_BIT(1, 2));
					m_v_door_lock_rr->SetValue(CAN_BIT(1, 1));
					m_v_door_lock_rl->SetValue(CAN_BIT(1, 3));
				}
			}
			break;

		case 0xbc04:
			if (mlframe == 1)
			{
				m_v_seat_belt_back_middle->SetValue(CAN_BIT(4, 3));

				StdMetrics.ms_v_env_headlights->SetValue(CAN_BIT(5, 4));

				/*
				 * Car data is by driver, but metrics is left/right
				 */
				if (IsLHD())
				{
					m_v_door_lock_fl->SetValue(CAN_BIT(1, 3));
					m_v_door_lock_fr->SetValue(CAN_BIT(1, 2));
					m_v_seat_belt_back_left->SetValue(CAN_BIT(4, 2));
					m_v_seat_belt_back_right->SetValue(CAN_BIT(4, 4));
				}
				else
				{
					m_v_door_lock_fr->SetValue(CAN_BIT(1, 3));
					m_v_door_lock_fl->SetValue(CAN_BIT(1, 2));
					m_v_seat_belt_back_right->SetValue(CAN_BIT(4, 2));
					m_v_seat_belt_back_left->SetValue(CAN_BIT(4, 4));
				}
			}
			break;

		case 0xbc07:
			if (mlframe == 1)
			{
				m_v_rear_defogger->SetValue(CAN_BIT(2, 1));
			}
			break;
		}
	}
}

/**
 * Determine if this car is a Hyundai Kona
 *
 * Currently from VIN
 */
bool OvmsVehicleKiaNiroEv::IsKona()
{
	if (m_vin[0] == 'K' && m_vin[1] == 'M' && m_vin[2] == 'H')
	{
		return true;
	}
	else
	{
		return false;
	}
}

/**
 * Get console ODO units
 *
 * Currently from configuration
 */
metric_unit_t OvmsVehicleKiaNiroEv::GetConsoleUnits()
{
	return MyConfig.GetParamValueBool("xkn", "consoleKilometers", true) ? Kilometers : Miles;
}

/**
 * Determine if this car is left hand drive
 *
 * Currently from configuration
 */
bool OvmsVehicleKiaNiroEv::IsLHD()
{
	return MyConfig.GetParamValueBool("xkn", "leftDrive", true);
}
