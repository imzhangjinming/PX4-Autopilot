/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

#include "JetControl.hpp"

JetControl::JetControl() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::test1)
{
}

JetControl::~JetControl()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool JetControl::init()
{
	// // execute Run() on every sensor_accel publication
	// if (!_sensor_accel_sub.registerCallback()) {
	// 	PX4_ERR("callback registration failed");
	// 	return false;
	// }

	// execute Run() on every manual_control_setpoint publication
	if (!_manual_control_setpoint_sub.registerCallback()){
		PX4_ERR("callback registration failed");
		return false;
	}

	// alternatively, Run on fixed interval
	// ScheduleOnInterval(5000_us); // 2000 us interval, 200 Hz rate

	return true;
}

void JetControl::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);

	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams(); // update module parameters (in DEFINE_PARAMETERS)
	}


	// Example
	//  update vehicle_status to check arming state
	if (_vehicle_status_sub.updated()) {
		vehicle_status_s vehicle_status;

		if (_vehicle_status_sub.copy(&vehicle_status)) {

			const bool armed = (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);

			if (armed && !_armed) {
				PX4_WARN("vehicle armed due to %d", vehicle_status.latest_arming_reason);

			} else if (!armed && _armed) {
				PX4_INFO("vehicle disarmed due to %d", vehicle_status.latest_disarming_reason);
			}

			_armed = armed;
		}
	}


	// // Example
	// //  grab latest accelerometer data
	// if (_sensor_accel_sub.updated()) {
	// 	sensor_accel_s accel;

	// 	if (_sensor_accel_sub.copy(&accel)) {
	// 		// DO WORK

	// 		// access parameter value (SYS_AUTOSTART)
	// 		if (_param_sys_autostart.get() == 1234) {
	// 			// do something if SYS_AUTOSTART is 1234
	// 		}
	// 	}
	// }

	if (_manual_control_setpoint_sub.updated()) {
		manual_control_setpoint_s manual_control_setpoint;
		if (_manual_control_setpoint_sub.copy(&manual_control_setpoint)) {
			actuator_jets_s actuator_jets{};
			if (_param_jet_use_rc_aux.get() != 0) {
				float aux_val;
				switch(_param_jet_use_rc_aux.get()){
					case 1:
						aux_val = manual_control_setpoint.aux1;break;
					case 2:
						aux_val = manual_control_setpoint.aux2;break;
					case 3:
						aux_val = manual_control_setpoint.aux3;break;
					case 4:
						aux_val = manual_control_setpoint.aux4;break;
					case 5:
						aux_val = manual_control_setpoint.aux5;break;
					case 6:
						aux_val = manual_control_setpoint.aux6;break;
					default:
						aux_val = -1;
						PX4_ERR("invalid rc aux number");
				}

				for (size_t i = 0; i < actuator_jets_s::NUM_CONTROLS; i++){
					actuator_jets.control[i] = aux_val;
				}

				// TODO:
				//	check vehicle arm state;
				//	enable jets publish in prearmed state;

				actuator_jets.timestamp = hrt_absolute_time();
				actuator_jets.timestamp_sample = actuator_jets.timestamp;
				_actuator_jets_pub.publish(actuator_jets);
			}else{
				// do nothing
			}
		}
	}

	// // Example
	// //  publish some data
	// orb_test_s data{};
	// data.val = 314159;
	// data.timestamp = hrt_absolute_time();
	// _orb_test_pub.publish(data);


	perf_end(_loop_perf);
}

int JetControl::task_spawn(int argc, char *argv[])
{
	JetControl *instance = new JetControl();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int JetControl::print_status()
{
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	return 0;
}

int JetControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int JetControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Example of a simple module running out of a work queue.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("jet_control", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int jet_control_main(int argc, char *argv[])
{
	return JetControl::main(argc, argv);
}
