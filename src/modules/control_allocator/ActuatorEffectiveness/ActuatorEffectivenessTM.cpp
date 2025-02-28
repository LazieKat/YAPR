////    CUSTOM CODE    ////

#include "ActuatorEffectivenessTM.hpp"


void ActuatorEffectivenessTM::getParam(const char * name, float * value)
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

ActuatorEffectivenessTM::ActuatorEffectivenessTM(ModuleParams *parent) : ModuleParams(parent)
{
	PX4_INFO("Servos Parameters:");

	for(int i = 0; i < MOTOR_COUNT; i++)
	{
		char name[20];
		sprintf(name, "CA_ROTOR%d_PX", i);
		getParam(name, &this->position_x[i]);
		sprintf(name, "CA_ROTOR%d_PY", i);
		getParam(name, &this->position_y[i]);
		sprintf(name, "CA_ROTOR%d_PZ", i);
		getParam(name, &this->position_z[i]);

		sprintf(name, "CA_ROTOR%d_AX", i);
		getParam(name, &this->axis_x[i]);
		sprintf(name, "CA_ROTOR%d_AY", i);
		getParam(name, &this->axis_y[i]);
		sprintf(name, "CA_ROTOR%d_AZ", i);
		getParam(name, &this->axis_z[i]);

		sprintf(name, "CA_ROTOR%d_CT", i);
		getParam(name, &this->thrust_coef[i]);
		sprintf(name, "CA_ROTOR%d_KM", i);
		getParam(name, &this->moment_ratio[i]);
	}

	int32_t tilt_axis;

	param_t param = param_find("CA_TM_AXIS");
	if(param == PARAM_INVALID)
	{
		return;
	}

	if(param_get(param, &tilt_axis) != PX4_OK)
	{
		return;
	}

	switch (tilt_axis)
	{
		case TiltAxis::DUAL:
			_tilt_axis = TiltAxis::DUAL;
			break;
		case TiltAxis::SINGLE_LATERAL:
			_tilt_axis = TiltAxis::SINGLE_LATERAL;
			break;
		case TiltAxis::SINGLE_BILATERAL:
			_tilt_axis = TiltAxis::SINGLE_BILATERAL;
			break;
		default:
			_tilt_axis = TiltAxis::SINGLE_LATERAL;
			break;
	}
}


// matrix::Vector3f
// ActuatorEffectivenessTM::arbit_rot(matrix::Vector3f a, matrix::Vector3f P, float theta)
// {
// 	matrix::Vector3f result;

// 	float d = cos(theta);
// 	float n = a.norm();
// 	auto  c = P.cross(a);
// 	float q = n * (float) sin(theta) / c.norm();

// 	auto w = matrix::Vector3f(d * a(0), d * a(1), d * a(2));
// 	auto z = matrix::Vector3f(q * c(0), q * c(1), q * c(2));
// 	auto v = w + z;

// 	result = matrix::Vector3f(n * v(0), n * v(1), n * v(2));
// 	return result;
// }


bool ActuatorEffectivenessTM::getEffectivenessMatrix(Configuration &configuration, EffectivenessUpdateReason external_update)
{
	if (external_update == EffectivenessUpdateReason::NO_EXTERNAL_UPDATE) {
		return false;
	}

	// Motors
	// const bool rotors_added_successfully = _mc_rotors.addActuators(configuration);

	// return rotors_added_successfully;

	configuration.num_actuators_matrix[0] = MOTOR_COUNT + SERVO_COUNT;
	configuration.num_actuators_matrix[1] = 0;
	configuration.trim[0].zero();
	configuration.linearization_point[0].zero();
	configuration.selected_matrix = 0;
	configuration.num_actuators[0] = MOTOR_COUNT;
	configuration.num_actuators[1] = SERVO_COUNT;

	for (size_t i = 0; i < NUM_ACTUATORS * MAX_NUM_MATRICES; i++)
	{
		configuration.matrix_selection_indexes[i] = 0;
	}

	matrix::Vector3f rotor_position[MOTOR_COUNT];
	matrix::Vector3f rotor_axis_thrust[MOTOR_COUNT];
	matrix::Vector3f rotor_axis_lateral[MOTOR_COUNT];
	matrix::Vector3f rotor_axis_bilateral[MOTOR_COUNT];
	matrix::Vector3f moments_x[MOTOR_COUNT];
	matrix::Vector3f moments_y[MOTOR_COUNT];
	matrix::Vector3f moments_z[MOTOR_COUNT];

	for (int i = 0; i < MOTOR_COUNT; i++)
	{
		rotor_position[i]       = matrix::Vector3f(position_x[i], position_y[i], position_z[i]);
		rotor_axis_thrust[i]    = matrix::Vector3f(axis_x[i], axis_y[i], axis_z[i]);
		rotor_axis_bilateral[i] = rotor_position[i].normalized();
		rotor_axis_lateral[i]   = rotor_axis_thrust[i].cross(rotor_axis_bilateral[i]);

		if(_tilt_axis == TiltAxis::SINGLE_LATERAL)
		{
			rotor_axis_bilateral[i].setZero();
		}
		else if(_tilt_axis == TiltAxis::SINGLE_BILATERAL)
		{
			rotor_axis_lateral[i].setZero();
		}

		moments_x[i] = rotor_position[i].cross(rotor_axis_bilateral[i]) - rotor_axis_bilateral[i] * moment_ratio[i];
		moments_y[i] = rotor_position[i].cross(rotor_axis_lateral[i])   - rotor_axis_lateral[i] * moment_ratio[i];
		moments_z[i] = rotor_position[i].cross(rotor_axis_thrust[i])    - rotor_axis_thrust[i] * moment_ratio[i];

		configuration.effectiveness_matrices[0](0, 0 + i*3) = thrust_coef[i] * moments_x[i](0);
		configuration.effectiveness_matrices[0](1, 0 + i*3) = thrust_coef[i] * moments_x[i](1);
		configuration.effectiveness_matrices[0](2, 0 + i*3) = thrust_coef[i] * moments_x[i](2);
		configuration.effectiveness_matrices[0](3, 0 + i*3) = thrust_coef[i] * rotor_axis_bilateral[i](0);
		configuration.effectiveness_matrices[0](4, 0 + i*3) = thrust_coef[i] * rotor_axis_bilateral[i](1);
		configuration.effectiveness_matrices[0](5, 0 + i*3) = thrust_coef[i] * rotor_axis_bilateral[i](2);

		configuration.effectiveness_matrices[0](0, 1 + i*3) = thrust_coef[i] * moments_y[i](0);
		configuration.effectiveness_matrices[0](1, 1 + i*3) = thrust_coef[i] * moments_y[i](1);
		configuration.effectiveness_matrices[0](2, 1 + i*3) = thrust_coef[i] * moments_y[i](2);
		configuration.effectiveness_matrices[0](3, 1 + i*3) = thrust_coef[i] * rotor_axis_lateral[i](0);
		configuration.effectiveness_matrices[0](4, 1 + i*3) = thrust_coef[i] * rotor_axis_lateral[i](1);
		configuration.effectiveness_matrices[0](5, 1 + i*3) = thrust_coef[i] * rotor_axis_lateral[i](2);

		configuration.effectiveness_matrices[0](0, 2 + i*3) = thrust_coef[i] * moments_z[i](0);
		configuration.effectiveness_matrices[0](1, 2 + i*3) = thrust_coef[i] * moments_z[i](1);
		configuration.effectiveness_matrices[0](2, 2 + i*3) = thrust_coef[i] * moments_z[i](2);
		configuration.effectiveness_matrices[0](3, 2 + i*3) = thrust_coef[i] * rotor_axis_thrust[i](0);
		configuration.effectiveness_matrices[0](4, 2 + i*3) = thrust_coef[i] * rotor_axis_thrust[i](1);
		configuration.effectiveness_matrices[0](5, 2 + i*3) = thrust_coef[i] * rotor_axis_thrust[i](2);
	}


	for (size_t i = 0; i < NUM_AXES ; i++)
	{
		for (size_t j = MOTOR_COUNT * 3; j < NUM_ACTUATORS; j++)
		{
			configuration.effectiveness_matrices[0](i, j) = 0;
		}
	}

	return true;
}


////    END OF CUSTOM CODE    ////
