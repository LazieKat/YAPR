////    CUSTOM CODE    ////

#pragma once

#include "ActuatorEffectiveness.hpp"
#include "ActuatorEffectivenessRotors.hpp"

class ActuatorEffectivenessTM : public ModuleParams, public ActuatorEffectiveness
{
public:
	ActuatorEffectivenessTM(ModuleParams *parent) : ModuleParams(parent) {};
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

	void allocate(matrix::Vector<float, NUM_AXES> c, matrix::Vector<float, NUM_ACTUATORS> &actuator_sp);

	matrix::Vector<float, NUM_ACTUATORS> _actuator_sp;
};


////   END OF CUSTOM CODE    ////
