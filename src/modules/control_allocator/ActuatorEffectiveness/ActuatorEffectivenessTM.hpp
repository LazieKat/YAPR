////    CUSTOM CODE    ////

#pragma once

#include "ActuatorEffectiveness.hpp"
#include "ActuatorEffectivenessRotors.hpp"

class ActuatorEffectivenessTM : public ModuleParams, public ActuatorEffectiveness
{
public:
	ActuatorEffectivenessTM(ModuleParams *parent) ;

	virtual ~ActuatorEffectivenessTM() = default;

	bool getEffectivenessMatrix(Configuration &configuration, EffectivenessUpdateReason external_update) override;

	void getDesiredAllocationMethod(AllocationMethod allocation_method_out[MAX_NUM_MATRICES]) const override
	{
		allocation_method_out[0] = AllocationMethod::TM;
	}

	void getNormalizeRPY(bool normalize[MAX_NUM_MATRICES]) const override
	{
		normalize[0] = true;
	}

	const char *name() const override { return "TM"; }

	void allocate(matrix::Vector<float, NUM_AXES> c, ActuatorEffectiveness::ActuatorVector &actuator_sp, ActuatorEffectiveness::ActuatorVector &servo_sp);

	matrix::Vector<float, NUM_ACTUATORS> _actuator_sp;

	float deg2pwm(float deg, int servo_num);

	void getParam(const char * name, float * value);

	static const uint8_t SERVO_COUNT = 4;

	float min[SERVO_COUNT],
              max[SERVO_COUNT],
	      mec_min[SERVO_COUNT],
	      mec_max[SERVO_COUNT],
	      trim[SERVO_COUNT];
};


////   END OF CUSTOM CODE    ////
