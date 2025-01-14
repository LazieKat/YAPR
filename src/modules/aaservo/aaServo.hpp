
#include <uORB/topics/actuator_servos.h>
#include <uORB/topics/actuator_servos_trim.h>
#include <uORB/Publication.hpp>


class aaServo
{
	public:
	void init();
	void publishServo(float value, int servo_num);
	float deg2pwm(float deg, int servo_num);

	uORB::Publication<actuator_servos_s>	_actuator_servos_pub{ORB_ID(actuator_servos)};
	uORB::Publication<actuator_servos_trim_s>	_actuator_servos_trim_pub{ORB_ID(actuator_servos_trim)};

	bool _armed{false};
	hrt_abstime _last_run{0};
	hrt_abstime _timestamp_sample{0};
	hrt_abstime _last_status_pub{0};

	static const uint8_t SERVO_COUNT = 4;

	float min[SERVO_COUNT],
              max[SERVO_COUNT],
	      mec_min[SERVO_COUNT],
	      mec_max[SERVO_COUNT],
	      trim[SERVO_COUNT];
};
