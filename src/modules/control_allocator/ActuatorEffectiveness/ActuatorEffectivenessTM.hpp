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

	const char *name() const override { return "TM"; }

private:
	static constexpr int NUM_ACTUATORS_MAX = 4;
	struct ParamHandles {
		param_t position_x;
		param_t position_y;
		param_t position_z;
		param_t axis_x;
		param_t axis_y;
		param_t axis_z;
		param_t thrust_coef;
		param_t moment_ratio;
	};
	ParamHandles _param_handles[NUM_ACTUATORS_MAX];

};


////   END OF CUSTOM CODE    ////
