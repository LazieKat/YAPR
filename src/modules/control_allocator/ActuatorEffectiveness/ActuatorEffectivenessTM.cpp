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


float ActuatorEffectivenessTM::deg2pwm(float deg, int servo_num)
{
	// restrain to mechanical limit
	if(deg < mec_min[servo_num]) deg = mec_min[servo_num];
	if(deg > mec_max[servo_num]) deg = mec_max[servo_num];

	deg = deg + trim[servo_num];

	// map min max to -1 to 1 and find the value for deg
	float value = (deg - min[servo_num]) / (max[servo_num] - min[servo_num]) * 2 - 1;
	return value;
}

bool ActuatorEffectivenessTM::getEffectivenessMatrix(Configuration &configuration, EffectivenessUpdateReason external_update)
{
	if (external_update == EffectivenessUpdateReason::NO_EXTERNAL_UPDATE) {
		return false;
	}

	// Motors
	// const bool rotors_added_successfully = _mc_rotors.addActuators(configuration);

	// return rotors_added_successfully;

	configuration.num_actuators_matrix[0] = 4;
	configuration.num_actuators_matrix[1] = 0;
	configuration.trim[0].zero();
	configuration.linearization_point[0].zero();
	configuration.selected_matrix = 0;
	configuration.num_actuators[0] = 4;
	configuration.num_actuators[1] = 0;

	for (size_t i = 0; i < NUM_ACTUATORS * MAX_NUM_MATRICES; i++)
	{
		configuration.matrix_selection_indexes[i] = 0;
	}

	// fill in the effectiveness matrix which is 6x16, but columns after 4 are zero
	float effectiveness_matrix[6][12] = {
		{ 0.7071 ,  0.7071  ,  0       , -0.7071  , -0.7071  ,  0       ,  0.7071  , -0.7071  ,  0       , -0.7071  ,  0.7071  ,  0      } ,
		{ 0.7071 , -0.7071  ,  0       , -0.7071  ,  0.7071  ,  0       , -0.7071  , -0.7071  ,  0       ,  0.7071  ,  0.7071  ,  0      } ,
		{ 0      ,  0       , -1.0000  ,  0       ,  0       , -1.0000  ,  0       ,  0       , -1.0000  ,  0       ,  0       , -1.0000 } ,
		{ 0      ,  0       , -0.1803  ,  0       ,  0       ,  0.1803  ,  0       ,  0       ,  0.1803  ,  0       ,  0       , -0.1803 } ,
		{ 0      ,  0       ,  0.1803  ,  0       ,  0       , -0.1803  ,  0       ,  0       ,  0.1803  ,  0       ,  0       , -0.1803 } ,
		{ 0      , -0.2550  ,  0.0500  ,  0       , -0.2550  ,  0.0500  ,  0       , -0.2550  , -0.0500  ,  0       , -0.2550  , -0.0500 } ,
	};


	for (size_t i = 0; i < NUM_AXES; i++)
	{
		for (size_t j = 0; j < NUM_ACTUATORS; j++)
		{
			configuration.effectiveness_matrices[0](i, j) = j < 12 ? effectiveness_matrix[i][j] : 0;
		}
	}

	return true;
}


void ActuatorEffectivenessTM::allocate(matrix::Vector<float, NUM_AXES> c, ActuatorEffectiveness::ActuatorVector &actuator_sp, ActuatorEffectiveness::ActuatorVector &servo_sp)
{
	float mix[12][6] = {
		{  0.1768  ,  0.1768  ,  0       ,  0       ,  0       , -0.0000 } ,
		{  0.1768  , -0.1768  ,  0       ,  0       ,  0       , -0.9441 } ,
		{  0       ,  0       , -0.2500  , -1.3865  ,  1.3865  ,  0.1851 } ,
		{ -0.1768  , -0.1768  ,  0       ,  0       ,  0       , -0.0000 } ,
		{ -0.1768  ,  0.1768  ,  0       ,  0       ,  0       , -0.9441 } ,
		{  0       ,  0       , -0.2500  ,  1.3865  , -1.3865  ,  0.1851 } ,
		{  0.1768  , -0.1768  ,  0       ,  0       ,  0       ,  0.0000 } ,
		{ -0.1768  , -0.1768  ,  0       ,  0       ,  0       , -0.9441 } ,
		{  0       ,  0       , -0.2500  ,  1.3865  ,  1.3865  , -0.1851 } ,
		{ -0.1768  ,  0.1768  ,  0       ,  0       ,  0       , -0.0000 } ,
		{  0.1768  ,  0.1768  ,  0       ,  0       ,  0       , -0.9441 } ,
		{  0       ,  0       , -0.2500  , -1.3865  , -1.3865  , -0.1851 } ,
	};

	matrix::Matrix<float, 12, 6> _mix;

	for (size_t i = 0; i < 12; i++)
	{
		for (size_t j = 0; j < 6; j++)
		{
			_mix(i, j) = mix[i][j];
		}
	}

	matrix::Vector<float, 12> res;

	res = _mix * c;

	for (size_t i = 0; i < 4; i++)
	{
		int idx = i * 3;

		float deg = -atan2f(res(idx+1), res(idx + 2)) * 57.29578f;
		servo_sp(i) = deg2pwm(deg, i);

		float act = sqrtf(res(idx + 1) * res(idx + 1) + res(idx + 2) * res(idx + 2));

		actuator_sp(i) = act > 1.0f ? 1.0f : act;

	}



}


////    END OF CUSTOM CODE    ////
