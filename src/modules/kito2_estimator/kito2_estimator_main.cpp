/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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

#include <px4_log.h>
#include <px4_posix.h>
#include <px4_module.h>
#include <px4_tasks.h>
#include <px4_posix.h>

#include "EstState.h"

#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/sensor_combined.h>

extern "C" __EXPORT int kito2_estimator_main(int argc, char *argv[]);

class Kito2Estimator : public ModuleBase<Kito2Estimator>
{
public:
	Kito2Estimator();

	virtual ~Kito2Estimator() = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static Kito2Estimator *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;

	EstState est;

private:

	void setArmState();

	int _status_sub {-1};

	vehicle_status_s vehicle_status = {};

	bool vehicle_status_updated = false;

	ArmState armState {ArmState::init};

	sensor_combined_s sensors = {};

	int _sensors_sub {-1};
	int _gyro_sub {-1};
	int _accel_sub {-1};

	float gyro[3] {0.0, 0.0, 0.0};
	float gyro_dt {0.0};

	float acc[3] {0.0, 0.0, 0.0};
	float acc_dt {0.0};

};

Kito2Estimator::Kito2Estimator()
{
  _status_sub = orb_subscribe(ORB_ID(vehicle_status));
  _sensors_sub = orb_subscribe(ORB_ID(sensor_combined));
}

int Kito2Estimator::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Attitude and position estimator using a Kalman Filter.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("kito2_estimator", "kito2_estimator");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int Kito2Estimator::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}


int Kito2Estimator::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("kito2_estimator",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_ESTIMATOR,
				      1024,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

Kito2Estimator *Kito2Estimator::instantiate(int argc, char *argv[])
{

	Kito2Estimator *instance = new Kito2Estimator();

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}



void Kito2Estimator::run()
{

  px4_pollfd_struct_t fds[1] = { {.fd = _sensors_sub, .events = POLLIN} };

    while (!should_exit()) {

        setArmState();

	// Create EstState and start est
	est.Cancel();
	est.Start();
	while (est.IsPollActive())
	  {
	      setArmState();

	      int ret = px4_poll(fds, sizeof(fds) / sizeof(fds[0]), 1000);

	      if (ret == 0) {
		  PX4_ERR("Time out");
		  break;
		} else if (ret < 0) {
		  PX4_ERR("ERROR return value : %d", ret);
		  break;
		} else {
		  orb_copy(ORB_ID(sensor_combined), _sensors_sub, &sensors);

                  gyro[0] = sensors.gyro_rad[0];
                  gyro[1] = sensors.gyro_rad[1];
                  gyro[2] = sensors.gyro_rad[2];
                  gyro_dt = sensors.gyro_integral_dt;
                  est.SetGyroData(gyro, gyro_dt);

                  acc[0] = sensors.accelerometer_m_s2[0];
                  acc[1] = sensors.accelerometer_m_s2[1];
                  acc[2] = sensors.accelerometer_m_s2[2];
                  acc_dt = sensors.accelerometer_integral_dt;
                  est.SetAccData(acc, acc_dt);
                }

	      est.Poll();
	  }
	break;
    }

}

void Kito2Estimator::setArmState()
{
    orb_check(_status_sub, &vehicle_status_updated);

    if (vehicle_status_updated) {
        if (orb_copy(ORB_ID(vehicle_status), _status_sub, &vehicle_status) == PX4_OK) {
            switch(vehicle_status.arming_state)
              {
              case vehicle_status_s::ARMING_STATE_INIT:
                if (armState != ArmState::init)
                  {
                    armState = ArmState::init;
                    PX4_INFO("vehicle_status:Init");
                    est.SetArmState(ArmState::init);
                  }
                break;
              case vehicle_status_s::ARMING_STATE_STANDBY:
                if (armState != ArmState::standby)
                  {
                    armState = ArmState::standby;
                    PX4_INFO("vehicle_status:Stanby");
                    est.SetArmState(ArmState::standby);
                  }
                break;
              case  vehicle_status_s::ARMING_STATE_ARMED:
                if (armState != ArmState::armed)
                  {
                    armState = ArmState::armed;
                    PX4_INFO("vehicle_status:Armed");
                    est.SetArmState(ArmState::armed);
                  }
                break;
              }
        }
    }
}


int kito2_estimator_main(int argc, char *argv[])
{
	return Kito2Estimator::main(argc, argv);
}
