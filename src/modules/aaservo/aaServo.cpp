#include "aaServo.hpp"
#include <uORB/topics/vehicle_command.h>
#include <px4_platform_common/log.h>
#include <parameters/param.h>

extern "C" __EXPORT int aaservo_main(int argc, char *argv[]);


void arm_px4()
{
    // Initialize vehicle_command message
    struct vehicle_command_s cmd{};
    cmd.timestamp = hrt_absolute_time();
    cmd.param1 = 1.0f; // Arm command (1 = arm, 0 = disarm)
    cmd.param2 = 0.0f; // Reserved (set to 0)
    cmd.command = vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM;
    cmd.target_system = 1;  // Target the main system
    cmd.target_component = 1; // Target the main component
    cmd.source_system = 1;   // Your system ID (1 is typical for the companion computer)
    cmd.source_component = 1; // Your component ID
    cmd.from_external = false; // Indicate the command originates from onboard

    // Advertise and publish the command
    orb_advert_t vehicle_command_pub = orb_advertise(ORB_ID(vehicle_command), &cmd);
    if (vehicle_command_pub == nullptr) {
        PX4_ERR("Failed to advertise vehicle_command");
    } else {
        PX4_INFO("Arming command sent.");
    }
}

void getParam(const char * name, float * value)
{
	param_t param = param_find(name);

	if(param == PARAM_INVALID)
	{
		PX4_ERR("Parameter %s not found", name);
		return;
	}

	if(param_get(param, value) != PX4_OK)
	{
		PX4_ERR("Failed to get parameter %s", name);
		return;
	}

	PX4_INFO("value of param %s is %f", name, (double) *value);
}

void aaServo::init()
{
	PX4_INFO("Servos Parameters:");

	for(int i = 0; i < SERVO_COUNT; i++)
	{
		char name[20];
		sprintf(name, "CA_SV_TL%d_MINA", i);
		getParam(name, &this->min[i]);
		sprintf(name, "CA_SV_TL%d_MAXA", i);
		getParam(name, &this->max[i]);
		sprintf(name, "CA_SV_TL%d_MINM", i);
		getParam(name, &this->mec_min[i]);
		sprintf(name, "CA_SV_TL%d_MAXM", i);
		getParam(name, &this->mec_max[i]);
		sprintf(name, "CA_SV_TL%d_TRM", i);
		getParam(name, &this->trim[i]);

		if(mec_min[i] < min[i]) this->mec_min[i] = min[i];
		if(mec_max[i] > max[i]) this->mec_max[i] = max[i];

		PX4_INFO("    Servo #%d: min: %f\tmax: %f\tmec_min: %f\tmec_max: %f\ttrim: %f",
				i+1, (double) min[i], (double) max[i], (double) mec_min[i], (double) mec_max[i], (double) trim[i]);
	}
}

float aaServo::deg2pwm(float deg, int servo_num)
{
	// restrain to mechanical limit
	if(deg < mec_min[servo_num]) deg = mec_min[servo_num];
	if(deg > mec_max[servo_num]) deg = mec_max[servo_num];

	deg = deg + trim[servo_num];

	// map min max to -1 to 1 and find the value for deg
	float value = (deg - min[servo_num]) / (max[servo_num] - min[servo_num]) * 2 - 1;
	return value;
}

void aaServo::publishServo(float value, int servo_num)
{
	arm_px4();

	float value_pwm = deg2pwm(value, servo_num);

	PX4_INFO("Publishing:\n        servo angle %f\n        pwm value %f\n        servo #%d",
			(double) value, (double) value_pwm, servo_num);

		actuator_servos_s servo{};
		servo.timestamp_sample = hrt_absolute_time();
		servo.timestamp = hrt_absolute_time();

		actuator_servos_trim_s trim_msg{};
		trim_msg.timestamp = servo.timestamp;


		for(int i = 0; i < 8; i++)
		{
			servo.control[i] = i == servo_num ? value_pwm : 0;
			trim_msg.trim[i] = 0;
		}

		_actuator_servos_pub.publish(servo);
		_actuator_servos_trim_pub.publish(trim_msg);


	PX4_INFO("Publication complete");
}


int aaservo_main(int argc, char *argv[])
{
	aaServo servo;

	servo.init();

	// take first argument as value and second as trim
	if(argc < 3)
	{
		PX4_ERR("Not Enough Arguments: Enter tilt in degrees and servo number.");
		return 1;
	}

	float value   = atof(argv[1]);
	int servo_num = atoi(argv[2]);

	servo.publishServo(value, servo_num);

	return OK;
}
