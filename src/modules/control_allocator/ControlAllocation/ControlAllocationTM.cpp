////    CUSTOM CODE    ////

#include "ControlAllocationTM.hpp"

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
	// Same scale on roll and pitch
	if (_normalize_rpy) {
		for (size_t axis_idx = 0; axis_idx <= 1; axis_idx++)
		{
			int num_non_zero_rp_torque = 0;

			for (int i = 0; i < _num_actuators; i++) {

				if (fabsf(_mix(i, axis_idx)) > 1e-5f) {
					++num_non_zero_rp_torque;
				}
			}

			float rp_norm_scale = 1.f;

			if (num_non_zero_rp_torque > 0) {
				rp_norm_scale = sqrtf(_mix.col(axis_idx).norm_squared() / (num_non_zero_rp_torque / 2.f));
			}

			_control_allocation_scale(axis_idx) = rp_norm_scale;
		}
	} else {
		_control_allocation_scale(0) = 1.f;
		_control_allocation_scale(1) = 1.f;
		_control_allocation_scale(2) = 1.f;
	}

	// Scale yaw separately
	_control_allocation_scale(2) = matrix::Vector3f(_mix(0,2), _mix(1,2), _mix(2,2)).norm();

	// Scale thrust by the sum of the individual thrust axes, and use the scaling for the Z axis if there's no actuators
	// (for tilted actuators)
	_control_allocation_scale(3) = 1.f;
	_control_allocation_scale(4) = 1.f;
	_control_allocation_scale(5) = 1.f;

	for (int axis_idx = 3; axis_idx <= 5; axis_idx++) {
		int num_non_zero_thrust = 0;
		float norm_sum = 0.f;

		for (int i = 0; i < _num_actuators; i++) {
			float norm = fabsf(_mix(i, axis_idx));

			if (norm > 1e-5f) {
				norm_sum += norm;
				++num_non_zero_thrust;
			}
		}

		if (num_non_zero_thrust > 0) {
			_control_allocation_scale(axis_idx) = norm_sum / num_non_zero_thrust;
		}
	}
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
			if (fabsf(_mix(i, j)) < 1e-5f) {
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


		float deg = -atan2f(_actuator_sp(idx+1), _actuator_sp(idx + 2)) * 57.29578f;
		servo_sp(i) = deg2pwm(deg, i);
	}

	for (size_t i = 0; i < _motor_count; i++)
	{
		_actuator_sp(i) = motor_sp(i);
		_actuator_sp(i + _servo_count) = servo_sp(i);
	}
}
