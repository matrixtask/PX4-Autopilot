/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "ActuatorEffectivenessStandardVTOL.hpp"

using namespace matrix;

ActuatorEffectivenessStandardVTOL::ActuatorEffectivenessStandardVTOL(ModuleParams *parent, bool decouple_tetra_mk7_em2_yaw)
	: ModuleParams(parent), _rotors(this), _control_surfaces(this),
	  _decouple_tetra_mk7_em2_yaw(decouple_tetra_mk7_em2_yaw)
{
}

bool
ActuatorEffectivenessStandardVTOL::getEffectivenessMatrix(Configuration &configuration,
		EffectivenessUpdateReason external_update)
{
	if (external_update == EffectivenessUpdateReason::NO_EXTERNAL_UPDATE) {
		return false;
	}

	// Motors
	configuration.selected_matrix = 0;
	_rotors.enablePropellerTorqueNonUpwards(true);
	const bool mc_rotors_added_successfully = _rotors.addActuators(configuration);

	if (_decouple_tetra_mk7_em2_yaw) {
		applyTeTraMk7EM2YawDecoupling(configuration);
	}

	_upwards_motors_mask = _rotors.getUpwardsMotors();
	_forwards_motors_mask = _rotors.getForwardsMotors();

	// Control Surfaces
	configuration.selected_matrix = 1;
	_first_control_surface_idx = configuration.num_actuators_matrix[configuration.selected_matrix];
	const bool surfaces_added_successfully = _control_surfaces.addActuators(configuration);

	return (mc_rotors_added_successfully && surfaces_added_successfully);
}

void
ActuatorEffectivenessStandardVTOL::applyTeTraMk7EM2YawDecoupling(ActuatorEffectiveness::Configuration &configuration)
{
	ActuatorEffectiveness::EffectivenessMatrix &effectiveness = configuration.effectiveness_matrices[0];

	// The Mk-7 EM2 lift rotors are laterally tilted. Keep yaw out of the
	// linear allocator because the physically useful yaw groups are not the
	// full geometric R+/R- sets.
	for (int rotor_index = 0; rotor_index < configuration.num_actuators_matrix[0]; ++rotor_index) {
		effectiveness(ControlAxis::YAW, rotor_index) = 0.f;
	}
}

void
ActuatorEffectivenessStandardVTOL::applyTeTraMk7EM2YawSetpoint(const matrix::Vector<float, NUM_AXES> &control_sp,
		ActuatorVector &actuator_sp, const ActuatorVector &actuator_min, const ActuatorVector &actuator_max) const
{
	static constexpr int yaw_positive_rotors[] = {1, 4, 6, 11};
	static constexpr int yaw_negative_rotors[] = {0, 5, 7, 10};

	const float yaw_sp = control_sp(ControlAxis::YAW);

	if (fabsf(yaw_sp) < 1e-5f) {
		return;
	}

	// Use only the thrust-increase group for each yaw direction:
	// R+ = rotors 2/5/7/12, R- = rotors 1/6/8/11 (1-based numbering).
	// The other same-yaw-sign lift rotors add a large roll moment
	// (for example, rotors 9/10 create P- during R+; the R- case is
	// handled symmetrically), so they stay out of this dedicated yaw mix.
	// Pulling down the opposite group can reintroduce roll through saturation
	// and desaturation, so it is intentionally not used for yaw authority.
	const int *yaw_rotors = yaw_sp > 0.f ? yaw_positive_rotors : yaw_negative_rotors;
	// control_sp(YAW) is already normalized by the rate controller. Map it to
	// a normalized motor increment instead of physical moment units.
	float yaw_delta = fabsf(yaw_sp) * _tetra_mk7_em2_yaw_mix_scale;

	for (int rotor_index_idx = 0; rotor_index_idx < 4; ++rotor_index_idx) {
		const int yaw_rotor = yaw_rotors[rotor_index_idx];

		if (PX4_ISFINITE(actuator_sp(yaw_rotor))) {
			yaw_delta = math::min(yaw_delta, actuator_max(yaw_rotor) - actuator_sp(yaw_rotor));
		}
	}

	if (yaw_delta <= 0.f) {
		return;
	}

	for (int rotor_index_idx = 0; rotor_index_idx < 4; ++rotor_index_idx) {
		const int yaw_rotor = yaw_rotors[rotor_index_idx];

		if (PX4_ISFINITE(actuator_sp(yaw_rotor))) {
			actuator_sp(yaw_rotor) += yaw_delta;
		}
	}
}

void ActuatorEffectivenessStandardVTOL::allocateAuxilaryControls(const float dt, int matrix_index,
		ActuatorVector &actuator_sp)
{
	if (matrix_index == 1) {
		// apply flaps
		normalized_unsigned_setpoint_s flaps_setpoint;

		if (_flaps_setpoint_sub.copy(&flaps_setpoint)) {
			_control_surfaces.applyFlaps(flaps_setpoint.normalized_setpoint, _first_control_surface_idx, dt, actuator_sp);
		}

		// apply spoilers
		normalized_unsigned_setpoint_s spoilers_setpoint;

		if (_spoilers_setpoint_sub.copy(&spoilers_setpoint)) {
			_control_surfaces.applySpoilers(spoilers_setpoint.normalized_setpoint, _first_control_surface_idx, dt, actuator_sp);
		}
	}
}

void ActuatorEffectivenessStandardVTOL::updateSetpoint(const matrix::Vector<float, NUM_AXES> &control_sp,
		int matrix_index, ActuatorVector &actuator_sp, const ActuatorVector &actuator_min, const ActuatorVector &actuator_max)
{
	if (matrix_index == 0) {
		if (_decouple_tetra_mk7_em2_yaw) {
			applyTeTraMk7EM2YawSetpoint(control_sp, actuator_sp, actuator_min, actuator_max);
		}

		stopMaskedMotorsWithZeroThrust(_forwards_motors_mask, actuator_sp);
	}
}

void ActuatorEffectivenessStandardVTOL::setFlightPhase(const FlightPhase &flight_phase)
{
	if (_flight_phase == flight_phase) {
		return;
	}

	ActuatorEffectiveness::setFlightPhase(flight_phase);

	// update stopped motors
	switch (flight_phase) {
	case FlightPhase::FORWARD_FLIGHT:
		_stopped_motors_mask |= _upwards_motors_mask;
		break;

	case FlightPhase::HOVER_FLIGHT:
	case FlightPhase::TRANSITION_FF_TO_HF:
	case FlightPhase::TRANSITION_HF_TO_FF:
		_stopped_motors_mask &= ~_upwards_motors_mask;
		break;
	}
}
