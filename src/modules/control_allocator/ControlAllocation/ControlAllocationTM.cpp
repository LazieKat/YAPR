////    CUSTOM CODE    ////

#include "ControlAllocationTM.hpp"

#include <math.h>

void
ControlAllocationTM::setEffectivenessMatrix(
	const matrix::Matrix<float, ControlAllocation::NUM_AXES, ControlAllocation::NUM_ACTUATORS> &effectiveness,
	const ActuatorVector &actuator_trim, const ActuatorVector &linearization_point, int num_actuators,
	bool update_normalization_scale)
{
	ControlAllocation::setEffectivenessMatrix(effectiveness, actuator_trim, linearization_point, num_actuators,
			update_normalization_scale);
	_mix_update_needed = true;
	_normalization_needs_update = update_normalization_scale;

	getParam("CA_TM_CUTOFF", &this->_tilt_cuttoff);


	for(uint i = 0; i < _servo_count; i++)
	{
		char name[20];
		sprintf(name, "CA_SV_TL%d_MINA", i);
		getParam(name, &this->_min[i]);
		sprintf(name, "CA_SV_TL%d_MAXA", i);
		getParam(name, &this->_max[i]);
		sprintf(name, "CA_SV_TL%d_MINM", i);
		getParam(name, &this->_mec_min[i]);
		sprintf(name, "CA_SV_TL%d_MAXM", i);
		getParam(name, &this->_mec_max[i]);
		sprintf(name, "CA_SV_TL%d_TRM", i);
		getParam(name, &this->_trim[i]);

		if(_mec_min[i] < _min[i]) this->_mec_min[i] = _min[i];
		if(_mec_max[i] > _max[i]) this->_mec_max[i] = _max[i];
	}
}

void
ControlAllocationTM::getParam(const char * name, float * value)
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

void
ControlAllocationTM::updatePseudoInverse()
{
	if (_mix_update_needed) {
		matrix::geninv(_effectiveness, _mix);

		if (_normalization_needs_update && !_had_actuator_failure) {
			updateControlAllocationMatrixScale();
			_normalization_needs_update = false;
		}

		normalizeControlAllocationMatrix();
		_mix_update_needed = false;
	}
}

void
ControlAllocationTM::updateControlAllocationMatrixScale()
{
	PX4_INFO(" ---------------------- updateControlAllocationMatrixScale --------------------------");

	// MOMENTS STUFF
	if (_normalize_rpy)
	{
		matrix::Vector<float, _motor_count> norm_roll;
		matrix::Vector<float, _motor_count> norm_pitch;
		matrix::Vector<float, _motor_count> norm_yaw;

		int nz_roll  = 0;
		int nz_pitch = 0;
		int nz_yaw   = 0;

		for (uint32_t i = 0; i < _motor_count; i++)
		{
			uint32_t idx = i * 3;

			norm_roll (i) = matrix::Vector3f(_mix(idx, 0), _mix(idx + 1, 0), _mix(idx + 2, 0)).norm();
			norm_pitch(i) = matrix::Vector3f(_mix(idx, 1), _mix(idx + 1, 1), _mix(idx + 2, 1)).norm();
			norm_yaw  (i) = matrix::Vector3f(_mix(idx, 2), _mix(idx + 1, 2), _mix(idx + 2, 2)).norm();

			norm_roll (i) = norm_roll (i) < eps ? 0 : norm_roll (i);
			norm_pitch(i) = norm_pitch(i) < eps ? 0 : norm_pitch(i);
			norm_yaw  (i) = norm_yaw  (i) < eps ? 0 : norm_yaw  (i);

			if (norm_roll (i) > 0) nz_roll++;
			if (norm_pitch(i) > 0) nz_pitch++;
			if (norm_yaw  (i) > 0) nz_yaw++;
		}

		_control_allocation_scale(0) = sqrtf(norm_roll.norm_squared() / (nz_roll/2));
		_control_allocation_scale(1) = sqrtf(norm_pitch.norm_squared() / (nz_pitch/2));
		_control_allocation_scale(2) = norm_yaw.max();
	}
	else
	{
		_control_allocation_scale(0) = 1.f;
		_control_allocation_scale(1) = 1.f;
		_control_allocation_scale(2) = 1.f;
	}


	// THRUST STUFF
	float norm_x_sum = 0;
	float norm_y_sum = 0;
	float norm_z_sum = 0;

	int nz_x = 0;
	int nz_y = 0;
	int nz_z = 0;

	for (size_t i = 0; i < 4; i++)
	{
		size_t idx = i * 3;

		float norm_x, norm_y, norm_z;

		norm_x = matrix::Vector3f(_mix(idx, 3), _mix(idx + 1, 3), _mix(idx + 2, 3)).norm();
		norm_y = matrix::Vector3f(_mix(idx, 4), _mix(idx + 1, 4), _mix(idx + 2, 4)).norm();
		norm_z = matrix::Vector3f(_mix(idx, 5), _mix(idx + 1, 5), _mix(idx + 2, 5)).norm();

		norm_x = norm_x < eps ? 0 : norm_x;
		norm_y = norm_y < eps ? 0 : norm_y;
		norm_z = norm_z < eps ? 0 : norm_z;

		if (norm_x > 0) nz_x++;
		if (norm_y > 0) nz_y++;
		if (norm_z > 0) nz_z++;

		norm_x_sum += norm_x;
		norm_y_sum += norm_y;
		norm_z_sum += norm_z;
	}

	_control_allocation_scale(3) = norm_x_sum / nz_x;
	_control_allocation_scale(4) = norm_y_sum / nz_y;
	_control_allocation_scale(5) = norm_z_sum / nz_z;
}

void
ControlAllocationTM::normalizeControlAllocationMatrix()
{
	PX4_INFO(" ---------------------- normalizeControlAllocationMatrix --------------------------");

	if (_control_allocation_scale(0) > FLT_EPSILON) {
		_mix.col(0) /= _control_allocation_scale(0);
		_mix.col(1) /= _control_allocation_scale(1);
	}

	if (_control_allocation_scale(2) > FLT_EPSILON) {
		_mix.col(2) /= _control_allocation_scale(2);
	}

	if (_control_allocation_scale(3) > FLT_EPSILON) {
		_mix.col(3) /= _control_allocation_scale(3);
		_mix.col(4) /= _control_allocation_scale(4);
		_mix.col(5) /= _control_allocation_scale(5);
	}

	// Set all the small elements to 0 to avoid issues
	// in the control allocation algorithms
	for (int i = 0; i < _num_actuators; i++) {
		for (int j = 0; j < NUM_AXES; j++) {
			if (fabsf(_mix(i, j)) < eps) {
				_mix(i, j) = 0.f;
			}
		}
	}
}

float
ControlAllocationTM::deg2pwm(float deg, int servo_num)
{
	// restrain to mechanical limit
	if(deg < _mec_min[servo_num]) deg = _mec_min[servo_num];
	if(deg > _mec_max[servo_num]) deg = _mec_max[servo_num];

	deg = deg + _trim[servo_num];

	// map min max to -1 to 1 and find the value for deg
	float value = (deg - _min[servo_num]) / (_max[servo_num] - _min[servo_num]) * 2 - 1;
	return value;
}

void
ControlAllocationTM::allocate()
{
	//Compute new gains if needed
	updatePseudoInverse();

	_prev_actuator_sp = _actuator_sp;

	// Allocate
	_actuator_sp = _mix * _control_sp;

	matrix::Vector<float, NUM_ACTUATORS> motor_sp;
	matrix::Vector<float, NUM_ACTUATORS> servo_sp;

	for (size_t i = 0; i < _servo_count; i++)
	{
		int idx = i * 3;

		float act = sqrtf(_actuator_sp(idx + 1) * _actuator_sp(idx + 1) + _actuator_sp(idx + 2) * _actuator_sp(idx + 2));
		motor_sp(i) = act > 1.0f ? 1.0f : act;


		float deg = 0;

		if (motor_sp(i) > _tilt_cuttoff)
		{
			deg = -atan2f(_actuator_sp(idx+1), _actuator_sp(idx + 2)) * 57.29578f;
		}

		servo_sp(i) = deg2pwm(deg, i);
	}

	for (size_t i = 0; i < _motor_count; i++)
	{
		_actuator_sp(i) = motor_sp(i);
		_actuator_sp(i + _servo_count) = servo_sp(i);
	}
}
