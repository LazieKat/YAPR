////    CUSTOM CODE    ////
#pragma once

#include "ControlAllocation.hpp"

class ControlAllocationTM: public ControlAllocation
{
public:
	ControlAllocationTM() {PX4_INFO("---------------------- ControlAllocationTM --------------------------");};
	virtual ~ControlAllocationTM() = default;

	void allocate() override;
	void setEffectivenessMatrix(const matrix::Matrix<float, NUM_AXES, NUM_ACTUATORS> &effectiveness,
				    const ActuatorVector &actuator_trim, const ActuatorVector &linearization_point, int num_actuators,
				    bool update_normalization_scale) override;

	virtual const char * name() const { return "TM"; }
protected:

	bool _mix_update_needed{false};

	/**
	 * Recalculate pseudo inverse if required.
	 *
	 */
	void updatePseudoInverse();

private:
	void normalizeControlAllocationMatrix();
	void updateControlAllocationMatrixScale();
	bool _normalization_needs_update{false};
};

////    END OF CUSTOM CODE    ////
