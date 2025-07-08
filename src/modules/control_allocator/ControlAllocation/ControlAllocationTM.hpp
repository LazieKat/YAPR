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


	matrix::Vector<float, NUM_AXES> getAllocatedControl() const
	{ return (_effectiveness * (_allocated_actuators)).emult(_control_allocation_scale); }

protected:

	bool _mix_update_needed{false};

	/**
	 * Recalculate pseudo inverse if required.
	 *
	 */
	void updatePseudoInverse();

private:
	float eps = 1e-5f;

	void getParam(const char * name, float * value);
	float deg2pwm(float deg, int servo_num);

	void applyPoly(matrix::Vector<float, NUM_ACTUATORS> &motor_sp);

	void normalizeControlAllocationMatrix();
	void updateControlAllocationMatrixScale();
	bool _normalization_needs_update{false};
	bool _is_normalized{true};

	static const uint _servo_count{4};
	static const uint _motor_count{4};
	static const uint _poly_deg{4};

	float _min[_servo_count];
	float _max[_servo_count];
	float _mec_min[_servo_count];
	float _mec_max[_servo_count];
	float _trim[_servo_count];
	float _tilt_cuttoff{0.0f};


	bool _has_poly{false};

	float _poly[_poly_deg] {}; // polynomial coefficients for thrust to pwm conversion

	// we need a seperate vector from _actuator_sp to store the actuator setpoints in a way
	// that can be multiplied with the effectiveness matrix
	matrix::Vector<float, NUM_ACTUATORS> _allocated_actuators{};

};

////    END OF CUSTOM CODE    ////
