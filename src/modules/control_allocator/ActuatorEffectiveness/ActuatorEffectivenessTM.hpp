////    CUSTOM CODE    ////

#pragma once

#include "ActuatorEffectiveness.hpp"
#include "ActuatorEffectivenessRotors.hpp"

class ActuatorEffectivenessTM : public ModuleParams, public ActuatorEffectiveness
{
public:
	ActuatorEffectivenessTM(ModuleParams *parent);

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

	virtual int numMatrices() const override{ return 1; }

	const char *name() const override { return "TM"; }

	typedef enum {
		DUAL = 0,
		SINGLE_LATERAL = 1,
		SINGLE_BILATERAL = 2
	} TiltAxis;

private:
	static constexpr int NUM_ACTUATORS_MAX = 4;

	void getParam(const char * name, float * value);
	// matrix::Vector3f arbit_rot(matrix::Vector3f a, matrix::Vector3f P, float theta);

	static const uint8_t SERVO_COUNT = 4;
	static const uint8_t MOTOR_COUNT = 4;

	TiltAxis _tilt_axis{SINGLE_LATERAL};

	float position_x[MOTOR_COUNT];
	float position_y[MOTOR_COUNT];
	float position_z[MOTOR_COUNT];
	float axis_x[MOTOR_COUNT];
	float axis_y[MOTOR_COUNT];
	float axis_z[MOTOR_COUNT];
	float thrust_coef[MOTOR_COUNT];
	float moment_ratio[MOTOR_COUNT];

};


////   END OF CUSTOM CODE    ////
