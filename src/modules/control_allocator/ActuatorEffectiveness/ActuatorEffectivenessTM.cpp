////    CUSTOM CODE    ////

#include "ActuatorEffectivenessTM.hpp"

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
	float effectiveness_matrix[6][16] = {
		{ -1.6250,  1.2350,  1.6250, -1.2350, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
		{  0.9750, -0.9750,  0.9750, -0.9750, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
		{  0.3250,  0.3250, -0.3250, -0.3250, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
		{       0,       0,       0,       0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
		{       0,       0,       0,       0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
		{ -6.5000, -6.5000, -6.5000, -6.5000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
	};


	for (size_t i = 0; i < NUM_AXES; i++)
	{
		for (size_t j = 0; j < NUM_ACTUATORS; j++)
		{
			configuration.effectiveness_matrices[0](i, j) = effectiveness_matrix[i][j];
		}
	}

	return true;
}


void ActuatorEffectivenessTM::allocate(matrix::Vector<float, NUM_AXES> c, matrix::Vector<float, NUM_ACTUATORS> &actuator_sp)
{
	float mix[16][6] = {
		{-0.4821,  0.7071,  0.7600, 0, 0, -1.0000},
		{ 0.4821, -0.7071,  1.0000, 0, 0, -1.0000},
		{ 0.4821,  0.7071, -0.7600, 0, 0, -1.0000},
		{-0.4821, -0.7071, -1.0000, 0, 0, -1.0000},
		{      0,       0,       0, 0, 0,       0},
		{      0,       0,       0, 0, 0,       0},
		{      0,       0,       0, 0, 0,       0},
		{      0,       0,       0, 0, 0,       0},
		{      0,       0,       0, 0, 0,       0},
		{      0,       0,       0, 0, 0,       0},
		{      0,       0,       0, 0, 0,       0},
		{      0,       0,       0, 0, 0,       0},
		{      0,       0,       0, 0, 0,       0},
		{      0,       0,       0, 0, 0,       0},
		{      0,       0,       0, 0, 0,       0},
		{      0,       0,       0, 0, 0,       0},
	};

	matrix::Matrix<float, NUM_ACTUATORS, NUM_AXES> _mix;

	for (size_t i = 0; i < NUM_ACTUATORS; i++)
	{
		for (size_t j = 0; j < NUM_AXES; j++)
		{
			_mix(i, j) = mix[i][j];
		}
	}

	actuator_sp = _mix * c;

	for (size_t i = 0; i < NUM_ACTUATORS; i++)
	{
		if (actuator_sp(i) < 0.0f)
		{
			actuator_sp(i) = 0.0f;
		}
		else if (actuator_sp(i) > 1.0f)
		{
			actuator_sp(i) = 1.0f;
		}
	}
}


////    END OF CUSTOM CODE    ////
