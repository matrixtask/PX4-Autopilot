/****************************************************************************
 *
 *   Copyright (c) 2015-2022 PX4 Development Team. All rights reserved.
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

/**
 * @file standard.cpp
 *
 * @author Simon Wilks		<simon@uaventure.com>
 * @author Roman Bapst		<bapstroman@gmail.com>
 * @author Andreas Antener	<andreas@uaventure.com>
 * @author Sander Smeets	<sander@droneslab.com>
 *
*/

#include "standard.h"
#include "vtol_att_control_main.h"

#include <float.h>

using namespace matrix;

Standard::Standard(VtolAttitudeControl *attc) :
	VtolType(attc)
{
}

void
Standard::parameters_update()
{
	VtolType::updateParams();

	// make sure that pusher ramp in backtransition is smaller than back transition (max) duration
	_param_vt_b_trans_ramp.set(math::min(_param_vt_b_trans_ramp.get(), _param_vt_b_trans_dur.get()));
}

void Standard::update_vtol_state()
{
	/* After flipping the switch the vehicle will start the pusher (or tractor) motor, picking up
	 * forward speed. After the vehicle has picked up enough speed the rotors shutdown.
	 * For the back transition the pusher motor is immediately stopped and rotors reactivated.
	 */

	float mc_weight = _mc_roll_weight;

	if (_vtol_vehicle_status->fixed_wing_system_failure) {
		// Failsafe event, engage mc motors immediately
		_vtol_mode = vtol_mode::MC_MODE;
		_pusher_throttle = 0.0f;

	} else if (!_attc->is_fixed_wing_requested()) {

		// the transition to fw mode switch is off
		if (_vtol_mode == vtol_mode::MC_MODE) {
			// in mc mode
			_vtol_mode = vtol_mode::MC_MODE;
			mc_weight = 1.0f;

		} else if (_vtol_mode == vtol_mode::FW_MODE) {
			// Regular backtransition
			resetTransitionStates();
			_vtol_mode = vtol_mode::TRANSITION_TO_MC;

		} else if (_vtol_mode == vtol_mode::TRANSITION_TO_FW) {
			// failsafe back to mc mode
			_vtol_mode = vtol_mode::MC_MODE;
			mc_weight = 1.0f;
			_pusher_throttle = 0.0f;

		} else if (_vtol_mode == vtol_mode::TRANSITION_TO_MC) {
			// speed exit condition: use ground if valid, otherwise airspeed
			bool exit_backtransition_speed_condition = false;

			if (_local_pos->v_xy_valid) {
				const Dcmf R_to_body(Quatf(_v_att->q).inversed());
				const Vector3f vel = R_to_body * Vector3f(_local_pos->vx, _local_pos->vy, _local_pos->vz);
				exit_backtransition_speed_condition = vel(0) < _param_mpc_xy_cruise.get();

			} else if (PX4_ISFINITE(_attc->get_calibrated_airspeed())) {
				exit_backtransition_speed_condition = _attc->get_calibrated_airspeed() < _param_mpc_xy_cruise.get();
			}

			const bool exit_backtransition_time_condition = _time_since_trans_start > _param_vt_b_trans_dur.get();

			if (can_transition_on_ground() || exit_backtransition_speed_condition || exit_backtransition_time_condition) {
				_vtol_mode = vtol_mode::MC_MODE;
			}
		}

	} else {
		// the transition to fw mode switch is on
		if (_vtol_mode == vtol_mode::MC_MODE || _vtol_mode == vtol_mode::TRANSITION_TO_MC) {
			// start transition to fw mode
			/* NOTE: The failsafe transition to fixed-wing was removed because it can result in an
			 * unsafe flying state. */
			resetTransitionStates();
			_vtol_mode = vtol_mode::TRANSITION_TO_FW;

		} else if (_vtol_mode == vtol_mode::FW_MODE) {
			// in fw mode
			_vtol_mode = vtol_mode::FW_MODE;
			mc_weight = 0.0f;

		} else if (_vtol_mode == vtol_mode::TRANSITION_TO_FW) {

			if (isFrontTransitionCompleted()) {
				_vtol_mode = vtol_mode::FW_MODE;

				// don't set pusher throttle here as it's being ramped up elsewhere
				_trans_finished_ts = hrt_absolute_time();
			}
		}
	}

	_mc_roll_weight = mc_weight;
	_mc_pitch_weight = mc_weight;
	_mc_yaw_weight = mc_weight;
	_mc_throttle_weight = mc_weight;

	// map specific control phases to simple control modes
	switch (_vtol_mode) {
	case vtol_mode::MC_MODE:
		_common_vtol_mode = mode::ROTARY_WING;
		break;

	case vtol_mode::FW_MODE:
		_common_vtol_mode = mode::FIXED_WING;
		break;

	case vtol_mode::TRANSITION_TO_FW:
		_common_vtol_mode = mode::TRANSITION_TO_FW;
		break;

	case vtol_mode::TRANSITION_TO_MC:
		_common_vtol_mode = mode::TRANSITION_TO_MC;
		break;
	}
}

void Standard::update_transition_state()
{
	const hrt_abstime now = hrt_absolute_time();
	float mc_weight = 1.0f;

	VtolType::update_transition_state();

	// we get attitude setpoint from a multirotor flighttask if climbrate is controlled.
	// in any other case the fixed wing attitude controller publishes attitude setpoint from manual stick input.
	if (_v_control_mode->flag_control_climb_rate_enabled) {
		// we need the incoming (virtual) attitude setpoints (both mc and fw) to be recent, otherwise return (means the previous setpoint stays active)
		if (_mc_virtual_att_sp->timestamp < (now - 1_s) || _fw_virtual_att_sp->timestamp < (now - 1_s)) {
			return;
		}

		*_v_att_sp = *_mc_virtual_att_sp;

	} else {
		// we need a recent incoming (fw virtual) attitude setpoint, otherwise return (means the previous setpoint stays active)
		if (_fw_virtual_att_sp->timestamp < (now - 1_s)) {
			return;
		}

		*_v_att_sp = *_fw_virtual_att_sp;
		_v_att_sp->thrust_body[2] = -_fw_virtual_att_sp->thrust_body[0];
	}


	const Eulerf attitude_setpoint_euler(Quatf(_v_att_sp->q_d));
	float roll_body = attitude_setpoint_euler.phi();
	float pitch_body = attitude_setpoint_euler.theta();
	float yaw_body = attitude_setpoint_euler.psi();

	// AIRSPEED-SCHEDULED lift-handoff floor for the MC motor weight: the wing carries lift L(V) ~ V^2;
	// floor the rotors at exactly the deficit the wing cannot carry -> wing + rotor ~= weight, continuous.
	// floor = max(0, 1 - (CAS/VT_LIFT_HND_V)^2). ~0 at/above the lift=weight speed (FW transition untouched),
	// rises to 1 (hover) as speed bleeds. Falls back to the flat VT_B_RAMP_MIN when VT_LIFT_HND_V = 0.
	float lift_floor = _param_vt_b_ramp_min.get();
	{
		const float vL = _param_vt_lift_hnd_v.get();
		const float cas = _attc->get_calibrated_airspeed();

		if (vL > FLT_EPSILON && PX4_ISFINITE(cas) && cas < vL) {
			const float r = cas / vL;
			lift_floor = math::max(lift_floor, math::constrain(1.0f - r * r, 0.0f, 1.0f));
		}
	}

	if (_vtol_mode == vtol_mode::TRANSITION_TO_FW) {

		if (_v_control_mode->flag_control_climb_rate_enabled) {
			roll_body = Eulerf(Quatf(_fw_virtual_att_sp->q_d)).phi();
		}

		if (_param_vt_psher_slew.get() <= FLT_EPSILON) {
			// just set the final target throttle value
			_pusher_throttle = _param_vt_f_trans_thr.get();

		} else if (_pusher_throttle <= _param_vt_f_trans_thr.get()) {
			// ramp up throttle to the target throttle value
			const float dt = math::min((now - _last_time_pusher_transition_update) / 1e6f, 0.05f);
			_pusher_throttle = math::min(_pusher_throttle +
						     _param_vt_psher_slew.get() * dt, _param_vt_f_trans_thr.get());

			_last_time_pusher_transition_update = now;
		}

		_airspeed_trans_blend_margin = getTransitionAirspeed() - getBlendAirspeed();

		// do blending of mc and fw controls if a blending airspeed has been provided and the minimum transition time has passed
		if (_airspeed_trans_blend_margin > 0.0f &&
		    PX4_ISFINITE(_attc->get_calibrated_airspeed()) &&
		    _attc->get_calibrated_airspeed() > 0.0f &&
		    _attc->get_calibrated_airspeed() >= getBlendAirspeed() &&
		    _time_since_trans_start > getMinimumFrontTransitionTime()) {

			mc_weight = 1.0f - fabsf(_attc->get_calibrated_airspeed() - getBlendAirspeed()) /
				    _airspeed_trans_blend_margin;
			// time based blending when no airspeed sensor is set

		} else if (!PX4_ISFINITE(_attc->get_calibrated_airspeed())) {
			mc_weight = 1.0f - _time_since_trans_start / getMinimumFrontTransitionTime();
			mc_weight = math::constrain(2.0f * mc_weight, 0.0f, 1.0f);
		}

		// ramp up FW_PSP_OFF
		pitch_body = math::radians(_param_fw_psp_off.get()) * (1.0f - mc_weight);
		_v_att_sp->thrust_body[0] = _pusher_throttle;
		const Quatf q_sp(Eulerf(roll_body, pitch_body, yaw_body));
		q_sp.copyTo(_v_att_sp->q_d);
		// OVERLAP floor (airspeed-scheduled): keep the lift rotors at >= lift_floor during
		// TRANSITION_TO_FW so they carry exactly the lift the wing is losing as speed bleeds.
		mc_weight = math::constrain(mc_weight, lift_floor, 1.0f);

	} else if (_vtol_mode == vtol_mode::TRANSITION_TO_MC) {

		// continually increase mc attitude control as we transition back to mc mode.
		// Start the ramp from VT_B_RAMP_MIN (not 0) so the lift rotors give partial thrust
		// IMMEDIATELY and overlap with the still-flying wing -> no zero-thrust free-fall dip.
		if (_param_vt_b_trans_ramp.get() > FLT_EPSILON) {
			const float ramp_floor = lift_floor;   // airspeed-scheduled wing->rotor handoff floor
			mc_weight = ramp_floor + (1.0f - ramp_floor) * (_time_since_trans_start / _param_vt_b_trans_ramp.get());
		}

		mc_weight = math::constrain(mc_weight, 0.0f, 1.0f);

		if (_v_control_mode->flag_control_climb_rate_enabled) {
			// control backtransition deceleration using pitch.
			const float pitch_mc = Eulerf(Quatf(_mc_virtual_att_sp->q_d)).theta();

			// (2) GRADUAL WING UNLOAD: blend pitch FW->MC on the AIRSPEED lift schedule (lift_floor),
			// not the fast mc_weight ramp, so the wing AoA/lift hands off CONTINUOUSLY to the rotors as
			// speed bleeds (no abrupt pitch step -> no sudden lift loss -> no free-fall dip). The DLC
			// flaperon fills the residual gap. Enabled only when VT_LIFT_HND_V > 0.
			if (_param_vt_lift_hnd_v.get() > FLT_EPSILON) {
				const float pitch_fw = Eulerf(Quatf(_fw_virtual_att_sp->q_d)).theta();
				pitch_body = lift_floor * pitch_mc + (1.0f - lift_floor) * pitch_fw;

			} else {
				pitch_body = pitch_mc;
			}

			// blend roll setpoint between FW and MC
			const float roll_body_fw = Eulerf(Quatf(_fw_virtual_att_sp->q_d)).phi();
			// [BETA-KICK] A SLOW P9 roll-out (hold the +15 bank, level gradually) was TESTED to kill the adverse-
			// sideslip beta kick -> it made it WORSE (spike -71, beta 14.9): holding the bank longer through the
			// decel builds MORE sideslip. The fast mc_weight blend is the lesser evil. Reverted.
			// [P10-ENTRY-COORD TESTED, REVERTED to stock] The entry sideslip beta ~14 deg [the body yaw drifts
			// 268->278 off the ground track during the back-trans] drives the entry roll transient. TWO C++ fixes
			// were tried and did NOT reduce it: (1) yaw=ground-course override -> WORSE [the body yaw drifts
			// regardless -> a 14 deg yaw error the controller fought via roll -> flaperon saturated, roll-osc 3.2];
			// (2) scaling the FW roll-setpoint *0.35 -> beta unchanged [14.7], a +22 deg roll spike. The beta is a
			// deep back-trans dynamic [MC velocity-correction yaw + decel coupling], not a yaw-setpoint or FW-bank
			// effect. STOCK blend kept. Steady descent beta=0 via the OFFBOARD yaw=course [python p10diag].
			roll_body = mc_weight * roll_body + (1.0f - mc_weight) * roll_body_fw;
		}

		const Quatf q_sp(Eulerf(roll_body, pitch_body, yaw_body));

		q_sp.copyTo(_v_att_sp->q_d);

		// [DUTCH-ROLL] The pusher MUST be cut immediately here. A P9 rampdown was TESTED and is CATASTROPHIC:
		// keeping the pusher running through the high-AoA back-trans pitch-up makes the prop NORMAL FORCE (P-factor)
		// generate a LARGE yaw moment (yaw rate +1.9->-2.4 became +28 deg/s) -> Dutch roll amplified -> tumble to
		// -177 deg / 45 m/s crash. The instant cut REMOVES that prop-at-AoA yaw source and is the correct behavior.
		_pusher_throttle = 0.0f;
	}

	_mc_roll_weight = mc_weight;
	_mc_pitch_weight = mc_weight;
	_mc_yaw_weight = mc_weight;
	_mc_throttle_weight = mc_weight;
}

void Standard::update_mc_state()
{
	VtolType::update_mc_state();

	_pusher_throttle = VtolType::pusher_assist();
}

void Standard::update_fw_state()
{
	VtolType::update_fw_state();
}

/**
 * Prepare message to actuators with data from mc and fw attitude controllers. An mc attitude weighting will determine
 * what proportion of control should be applied to each of the control groups (mc and fw).
 */
void Standard::fill_actuator_outputs()
{
	_torque_setpoint_0->timestamp = hrt_absolute_time();
	_torque_setpoint_0->timestamp_sample = _vehicle_torque_setpoint_virtual_mc->timestamp_sample;
	_torque_setpoint_0->xyz[0] = 0.f;
	_torque_setpoint_0->xyz[1] = 0.f;
	_torque_setpoint_0->xyz[2] = 0.f;

	_torque_setpoint_1->timestamp = hrt_absolute_time();
	_torque_setpoint_1->timestamp_sample = _vehicle_torque_setpoint_virtual_fw->timestamp_sample;
	_torque_setpoint_1->xyz[0] = 0.f;
	_torque_setpoint_1->xyz[1] = 0.f;
	_torque_setpoint_1->xyz[2] = 0.f;

	_thrust_setpoint_0->timestamp = hrt_absolute_time();
	_thrust_setpoint_0->timestamp_sample = _vehicle_thrust_setpoint_virtual_mc->timestamp_sample;
	_thrust_setpoint_0->xyz[0] = 0.f;
	_thrust_setpoint_0->xyz[1] = 0.f;
	_thrust_setpoint_0->xyz[2] = 0.f;

	_thrust_setpoint_1->timestamp = hrt_absolute_time();
	_thrust_setpoint_1->timestamp_sample = _vehicle_thrust_setpoint_virtual_fw->timestamp_sample;
	_thrust_setpoint_1->xyz[0] = 0.f;
	_thrust_setpoint_1->xyz[1] = 0.f;
	_thrust_setpoint_1->xyz[2] = 0.f;

	switch (_vtol_mode) {
	case vtol_mode::MC_MODE:

		// MC actuators:
		_torque_setpoint_0->xyz[0] = _vehicle_torque_setpoint_virtual_mc->xyz[0];
		_torque_setpoint_0->xyz[1] = _vehicle_torque_setpoint_virtual_mc->xyz[1];
		_torque_setpoint_0->xyz[2] = _vehicle_torque_setpoint_virtual_mc->xyz[2];
		_thrust_setpoint_0->xyz[2] = _vehicle_thrust_setpoint_virtual_mc->xyz[2];

		// FW actuators:
		if (!_param_vt_elev_mc_lock.get()) {
			_torque_setpoint_1->xyz[0] = _vehicle_torque_setpoint_virtual_fw->xyz[0];
			_torque_setpoint_1->xyz[1] = _vehicle_torque_setpoint_virtual_fw->xyz[1];
		}

		_thrust_setpoint_0->xyz[0] = _pusher_throttle;
		_fw_mode_entry_time = 0; // reset the FW-handoff ramp while not in FW
		break;

	case vtol_mode::TRANSITION_TO_FW:

	// FALLTHROUGH
	case vtol_mode::TRANSITION_TO_MC:
		// MC actuators:
		_torque_setpoint_0->xyz[0] = _vehicle_torque_setpoint_virtual_mc->xyz[0] * _mc_roll_weight;
		_torque_setpoint_0->xyz[1] = _vehicle_torque_setpoint_virtual_mc->xyz[1] * _mc_pitch_weight;
		_torque_setpoint_0->xyz[2] = _vehicle_torque_setpoint_virtual_mc->xyz[2] * _mc_yaw_weight;
		_thrust_setpoint_0->xyz[2] = _vehicle_thrust_setpoint_virtual_mc->xyz[2] * _mc_throttle_weight;

		// FW actuators
		// [STEP1 RESULT] Pure-MC roll in the back-trans (FW roll zeroed) was TESTED -> roll spike got WORSE
		// (-72 vs -40): the lift rotors alone (5% thrust at 70 m/s) cannot control the Cl_beta Dutch roll; the
		// FW ailerons are ESSENTIAL and already saturated. So the spike is a real airframe mode, NOT a blend
		// artifact, and the FW+MC blend is optimal. Reverted to full FW roll.
		_torque_setpoint_1->xyz[0] = _vehicle_torque_setpoint_virtual_fw->xyz[0];
		_torque_setpoint_1->xyz[1] = _vehicle_torque_setpoint_virtual_fw->xyz[1];
		_torque_setpoint_1->xyz[2] = _vehicle_torque_setpoint_virtual_fw->xyz[2];
		_thrust_setpoint_0->xyz[0] = _pusher_throttle;
		_fw_mode_entry_time = 0; // reset the FW-handoff ramp while still transitioning

		break;

	case vtol_mode::FW_MODE:

		// Mk-7 SMOOTH lift->wing handoff: for the first ~2 s of FW, fade the lift-rotor attitude+thrust
		// contribution (ramp 1->0) instead of cutting it abruptly, so roll authority transfers gradually
		// to the ailerons. The abrupt stop momentarily exceeded the small v6 ailerons' roll authority and
		// caused a roll-over; this gradual fade bridges the gap. (Requires the upwards motors NOT be hard-
		// stopped at FW entry -- see ActuatorEffectivenessStandardVTOL::setFlightPhase.)
		if (_fw_mode_entry_time == 0) { _fw_mode_entry_time = hrt_absolute_time(); }

		{
			const float ramp = math::constrain(1.0f - (float)(hrt_absolute_time() - _fw_mode_entry_time) / 2.0e6f,
							   0.0f, 1.0f);
			_torque_setpoint_0->xyz[0] = _vehicle_torque_setpoint_virtual_mc->xyz[0] * ramp;
			_torque_setpoint_0->xyz[1] = _vehicle_torque_setpoint_virtual_mc->xyz[1] * ramp;
			_torque_setpoint_0->xyz[2] = _vehicle_torque_setpoint_virtual_mc->xyz[2] * ramp;
			_thrust_setpoint_0->xyz[2] = _vehicle_thrust_setpoint_virtual_mc->xyz[2] * ramp;
		}

		// Mk-7 ROTOR-ASSISTED STEEP CLIMB: command all lift rotors at a uniform collective in FW so the
		// rotors carry weight / add climb force (the design 1/4 climb is a low-speed rotor-borne climb; the
		// forward-thrust-limited pusher alone can only hold ~1/6). Symmetric collective = pure upward body
		// force, no moment -> the FW surfaces keep attitude. Body-z is +down, so up-thrust is negative.
		if (_param_vt_fw_mc_thr.get() > 0.01f) {
			_thrust_setpoint_0->xyz[2] = -_param_vt_fw_mc_thr.get();
		}

		// FW actuators
		_torque_setpoint_1->xyz[0] = _vehicle_torque_setpoint_virtual_fw->xyz[0];
		_torque_setpoint_1->xyz[1] = _vehicle_torque_setpoint_virtual_fw->xyz[1];
		_torque_setpoint_1->xyz[2] = _vehicle_torque_setpoint_virtual_fw->xyz[2];
		_thrust_setpoint_0->xyz[0] = _vehicle_thrust_setpoint_virtual_fw->xyz[0];
		break;
	}
}

void
Standard::waiting_on_tecs()
{
	// keep thrust from transition
	_v_att_sp->thrust_body[0] = _pusher_throttle;
};

void Standard::blendThrottleAfterFrontTransition(float scale)
{
	const float tecs_throttle = _v_att_sp->thrust_body[0];
	_v_att_sp->thrust_body[0] = scale * tecs_throttle + (1.0f - scale) * _pusher_throttle;
}
