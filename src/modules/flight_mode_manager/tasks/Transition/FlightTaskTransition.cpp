/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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
 * @file FlightTaskTransition.cpp
 */

#include "FlightTaskTransition.hpp"

#include <lib/geo/geo.h>
#include <uORB/topics/vehicle_attitude.h>

using namespace matrix;

FlightTaskTransition::FlightTaskTransition()
{
	param_get(param_find("FW_PSP_OFF"), &_param_fw_psp_off);
	param_get(param_find("VT_B_DEC_I"), &_param_vt_b_dec_i);
	param_get(param_find("VT_B_DEC_MSS"), &_param_vt_b_dec_mss);
	param_get(param_find("VT_F_TRANS_VZ"), &_param_vt_f_trans_vz);
	param_get(param_find("VT_B_TRANS_VZ"), &_param_vt_b_trans_vz);
	param_get(param_find("VT_B_TRANS_VZLO"), &_param_vt_b_trans_vzlo);
	param_get(param_find("VT_B_TRANS_HMIN"), &_param_vt_b_trans_hmin);
	param_get(param_find("VT_F_YAW_HOLD"), &_param_vt_f_yaw_hold);
	param_get(param_find("MPC_ACC_DECOUPLE"), &_param_mpc_acc_decouple);
	loadBrakeParams();
}

void FlightTaskTransition::loadBrakeParams()
{
	// VT_BFF_* (vtol_att_control_params.yaml). Model constants without a parameter keep the Params defaults.
	int32_t enabled = 0;
	param_get(param_find("VT_BFF_EN"), &enabled);
	_brake_ff_enabled = (enabled == 1);

	float deg = 0.f;
	param_get(param_find("VT_BFF_MASS"), &_brake_params.mass);
	param_get(param_find("VT_BFF_SCLA"), &_brake_params.scla);
	param_get(param_find("VT_BFF_SPCL"), &_brake_params.spcl);
	param_get(param_find("VT_BFF_SPCD"), &_brake_params.spcd);
	param_get(param_find("VT_BFF_CDA"), &_brake_params.cda);
	param_get(param_find("VT_BFF_KIND"), &_brake_params.kind);
	param_get(param_find("VT_BFF_RMIN"), &_brake_params.r_min);
	param_get(param_find("VT_LIFT_HND_V"), &_brake_params.v_lift);
	param_get(param_find("MPC_THR_HOVER"), &_hover_thrust);
	int32_t observer = 0;
	param_get(param_find("VT_BFF_LOBS"), &observer);
	_brake_lift_observer = (observer == 1) && _hover_thrust > 0.05f;

	if (param_get(param_find("VT_BFF_AMAX"), &deg) == PX4_OK) { _brake_params.alpha_max = math::radians(deg); }

	if (param_get(param_find("VT_BFF_PRATE"), &deg) == PX4_OK) { _brake_pitch_rate = math::radians(deg); }

	if (param_get(param_find("MPC_TILTMAX_AIR"), &deg) == PX4_OK) { _brake_params.pitch_max = math::radians(deg); }

	// the law needs a wing-borne speed reference; without one (VT_LIFT_HND_V 0) it would put all weight on the wing
	if (_brake_params.v_lift < 1.f || _brake_params.mass < 1.f) {
		_brake_ff_enabled = false;
	}
}

bool FlightTaskTransition::activate(const trajectory_setpoint_s &last_setpoint)
{
	bool ret = FlightTask::activate(last_setpoint);

	if (PX4_ISFINITE(last_setpoint.velocity[2])) {
		_vel_z_filter.reset(last_setpoint.velocity[2]);

	} else {
		_vel_z_filter.reset(_velocity(2));
	}

	const Vector2f acceleration_xy{_sub_vehicle_local_position.get().ax, _sub_vehicle_local_position.get().ay};
	_accel_filter.reset(acceleration_xy);

	if (_sub_vehicle_status.get().in_transition_to_fw) {
		_gear.landing_gear = landing_gear_s::GEAR_UP;
		_transition_yaw = _yaw; // navigator aligned the nose with the next waypoint (MIS_YAW_ERR) before this

	} else {
		_gear.landing_gear = landing_gear_s::GEAR_DOWN;

		// back transition: the brake pitch starts from the current (wing-borne) pitch, no step
		uORB::SubscriptionData<vehicle_attitude_s> attitude{ORB_ID(vehicle_attitude)};
		const matrix::Quatf q{attitude.get().q};
		_brake_pitch = q.isAllFinite() ? matrix::Eulerf(q).theta() : NAN;
		_brake_trim = 0.f;
		_brake_lift = 0.f;
		_brake_sink_max = INFINITY;
		_brake_time = 0.f;
		_brake_decel_filter.reset(0.f);
		_brake_params.lift_scale = 1.f;
		_brake_spoiler = 0.f;
	}

	return ret;
}

bool FlightTaskTransition::updateInitialize()
{
	bool ret = FlightTask::updateInitialize();
	_sub_vehicle_status.update();
	_sub_position_sp_triplet.update();
	_sub_airspeed.update();
	return ret;
}

bool FlightTaskTransition::update()
{
	// tailsitters will override attitude and thrust setpoint
	// tiltrotors and standard vtol will overrride roll and pitch setpoint but keep vertical thrust setpoint
	bool ret = FlightTask::update();

	// Vertical velocity: upstream holds altitude (target 0). With VT_F_TRANS_VZ / VT_B_TRANS_VZ set, the
	// transition climbs towards / descends towards the mission altitude instead (see computeVerticalVelocityTarget).
	const float vz_target = computeVerticalVelocityTarget();
	_velocity_setpoint(2) = _vel_z_filter.update(vz_target, _deltatime);

	// the default MPC_Z_VEL_MAX_DN (slow MC descent) must not cap the faster wing-borne descent
	if (vz_target > _constraints.speed_down) {
		_constraints.speed_down = vz_target;
	}

	// calculate a horizontal acceleration vector which corresponds to an attitude composed of pitch up by _param_fw_psp_off
	// and zero roll angle
	float tilt_setpoint = math::radians(_param_fw_psp_off);
	Vector2f horizontal_acceleration_direction;

	// Vertical acceleration feed-forward: g in the tilt computation unless MPC_ACC_DECOUPLE is off (see below)
	float vertical_specific_force = CONSTANTS_ONE_G;

	if (!_sub_vehicle_status.get().in_transition_to_fw) {
		const Vector2f velocity_xy{_velocity};
		horizontal_acceleration_direction = -velocity_xy.unit_or_zero();

		if (_brake_ff_enabled) {
			// Feed-forward brake: pitch from the force balance, wing lift fed to the rotors (they make W - L).
			tilt_setpoint = computeBackTransitionBrake();
			_acceleration_setpoint(2) = _brake_lift / _brake_params.mass; // NED: less rotor thrust by the wing lift

			if (_param_mpc_acc_decouple == 0) {
				vertical_specific_force = CONSTANTS_ONE_G - _acceleration_setpoint(2); // PositionControl tilt uses g - a_z
			}

		} else {
			tilt_setpoint = computeBackTransitionTiltSetpoint();
		}

	} else {
		// Forward transition: use heading direction
		horizontal_acceleration_direction = Dcm2f(_yaw) * Vector2f(-1.0f, 0.0f);
	}

	_acceleration_setpoint.xy() = tanf(tilt_setpoint) * vertical_specific_force * horizontal_acceleration_direction;

	// Upstream commands zero yaw rate only, so the heading drifts while the MC yaw authority is still needed
	// (Mk-7 EM3: +16 deg from the pusher reaction torque, then a 20 deg bank and 34 m sink at FW entry).
	// VT_F_YAW_HOLD holds the heading the front transition started with.
	if (_sub_vehicle_status.get().in_transition_to_fw && _param_vt_f_yaw_hold == 1 && PX4_ISFINITE(_transition_yaw)) {
		_yaw_setpoint = _transition_yaw;
		_yawspeed_setpoint = NAN;

	} else {
		_yaw_setpoint = NAN;
		_yawspeed_setpoint = 0.f;
	}

	return ret;
}

float FlightTaskTransition::computeVerticalVelocityTarget() const
{
	// NED: negative = climb. Returns 0 (hold altitude, upstream behaviour) whenever the feature is off or the
	// target altitude is unknown.
	const vehicle_local_position_s &lpos = _sub_vehicle_local_position.get();

	if (!lpos.z_global || !PX4_ISFINITE(_position(2))) {
		return 0.f;
	}

	const float altitude = lpos.ref_alt - _position(2); // AMSL

	if (_sub_vehicle_status.get().in_transition_to_fw) {
		// Front transition: climb towards the next waypoint altitude, slowing down when within reach.
		const position_setpoint_s &target = _sub_position_sp_triplet.get().current;

		if (_param_vt_f_trans_vz < FLT_EPSILON || !target.valid || !PX4_ISFINITE(target.alt)) {
			return 0.f;
		}

		const float climb = math::constrain((target.alt - altitude) / kAltitudeCaptureTimeConstant, 0.f, _param_vt_f_trans_vz);
		return -climb;
	}

	// Back transition: descend towards the landing point, but stay VT_B_TRANS_HMIN above it. The allowed sink
	// rate drops to VT_B_TRANS_VZLO at low speed (rotor vortex ring state), as in the Mk-7 route evaluation.
	// Navigator keeps "current" at the approach altitude during a VTOL landing; the landing point itself is "next"
	// (type LAND). Its `valid` flag is not always set during the transition item, so the type and the distance to
	// "current" (same point) are checked instead.
	const position_setpoint_s &current = _sub_position_sp_triplet.get().current;
	const position_setpoint_s &land = _sub_position_sp_triplet.get().next;

	if (_param_vt_b_trans_vz < FLT_EPSILON || land.type != position_setpoint_s::SETPOINT_TYPE_LAND
	    || !PX4_ISFINITE(land.alt) || !current.valid
	    || get_distance_to_next_waypoint(current.lat, current.lon, land.lat, land.lon) > 100.f) {
		return 0.f;
	}

	// Sink schedule over horizontal speed: VZLO below 30 kt (vortex ring state), blending to VZ at 50-60 kt, then
	// fading to 0 at 80 kt. Descending at speed adds the energy the braking has to remove: a wing-borne Mk-7
	// started at 72 m/s, descended 295 m at 7 m/s and was still at 72 m/s after 80 s (SITL HQ01 run 4).
	const float speed = matrix::Vector2f(_velocity).norm();
	const float blend = math::constrain((speed - kLowSpeedSink) / (kFullSpeedSink - kLowSpeedSink), 0.f, 1.f);
	float sink_limit = _param_vt_b_trans_vzlo + (_param_vt_b_trans_vz - _param_vt_b_trans_vzlo) * blend;

	if (_brake_ff_enabled) {
		// The brake law knows how much descent the braking can absorb (angle of attack left over).
		sink_limit = math::min(sink_limit, _brake_sink_max);

	} else {
		sink_limit *= 1.f - math::constrain((speed - kFullSinkMaxSpeed) / (kNoSinkSpeed - kFullSinkMaxSpeed), 0.f, 1.f);
	}

	const float floor_alt = land.alt + _param_vt_b_trans_hmin;
	return math::constrain((altitude - floor_alt) / kAltitudeCaptureTimeConstant, 0.f, sink_limit);
}

float FlightTaskTransition::computeBackTransitionDecelSetpoint() const
{
	// Deceleration that stops the vehicle at the back-transition end point: closing speed^2 / (2 distance).
	// Upstream used the distance along the velocity direction; at low speed a small lateral drift turns the velocity
	// away from the point, that distance collapses (HQ01: 13 m along-velocity with the pad 79 m away) and the demand
	// stopped the vehicle 80 m short, followed by a 7 m/s MC reposition with a pitch / roll weave.
	const Vector2f position_xy{_position};
	const Vector2f velocity_xy{_velocity};
	float deceleration_setpoint = _param_vt_b_dec_mss;

	const float max_hor_pos_uncertainty_limit = 10.f;

	if (_sub_vehicle_local_position.get().eph < max_hor_pos_uncertainty_limit
	    && _sub_position_sp_triplet.get().current.valid
	    && _sub_vehicle_local_position.get().xy_global && position_xy.isAllFinite() && velocity_xy.isAllFinite()) {
		Vector2f position_setpoint_local;
		_geo_projection.project(_sub_position_sp_triplet.get().current.lat, _sub_position_sp_triplet.get().current.lon,
					position_setpoint_local(0), position_setpoint_local(1));

		const Vector2f pos_to_target = position_setpoint_local - position_xy; // backtransition end-point w.r.t. vehicle
		const float dist_to_target = pos_to_target.norm();
		const float closing_speed = dist_to_target > FLT_EPSILON ? velocity_xy.dot(pos_to_target / dist_to_target) : 0.f;

		if (dist_to_target > FLT_EPSILON && closing_speed > FLT_EPSILON) {
			// Backtransition target point is ahead of the vehicle, compute the desired deceleration
			deceleration_setpoint = closing_speed * closing_speed / (2.f * dist_to_target);

		} else {
			deceleration_setpoint = 2.f * _param_vt_b_dec_mss;
		}

		deceleration_setpoint = math::min(deceleration_setpoint, 2.f * _param_vt_b_dec_mss);
	}

	return deceleration_setpoint;
}

float FlightTaskTransition::computeBackTransitionTiltSetpoint()
{
	const Vector2f velocity_xy{_velocity};
	const Vector2f velocity_xy_direction = velocity_xy.unit_or_zero();
	const float deceleration_setpoint = computeBackTransitionDecelSetpoint();

	const Vector2f acceleration_xy_raw{_sub_vehicle_local_position.get().ax, _sub_vehicle_local_position.get().ay};
	const Vector2f acceleration_xy = _accel_filter.update(acceleration_xy_raw, _deltatime);
	const float deceleration = -acceleration_xy.dot(velocity_xy_direction); // Zero when velocity invalid
	const float deceleration_error = deceleration_setpoint - deceleration;

	// Update back-transition deceleration error integrator
	_decel_error_bt_int += (_param_vt_b_dec_i * deceleration_error) * _deltatime;
	_decel_error_bt_int = math::constrain(_decel_error_bt_int, 0.f, kDecelerationIntegratorLimit);
	return _decel_error_bt_int;
}

float FlightTaskTransition::computeBackTransitionBrake()
{
	// Feed-forward brake (lib/vtol_brake): returns the pitch [rad]; sets the wing lift for the rotor feed-forward,
	// the sink limit for computeVerticalVelocityTarget() and publishes the setpoint for vtol_att_control (spoiler).
	const Vector2f velocity_xy{_velocity};

	if (!velocity_xy.isAllFinite() || !PX4_ISFINITE(_velocity(2))) {
		_brake_lift = 0.f;
		return computeBackTransitionTiltSetpoint();
	}

	_brake_time += _deltatime;
	const Vector2f velocity_xy_direction = velocity_xy.unit_or_zero();

	// Demand: stop at the end point, capped and jerk-limited (comfort), plus an integral trim for model error.
	const float decel_setpoint = math::min(computeBackTransitionDecelSetpoint(), kBrakeDecelCap * _param_vt_b_dec_mss,
					       kBrakeJerk * _brake_time);
	const Vector2f acceleration_xy{_sub_vehicle_local_position.get().ax, _sub_vehicle_local_position.get().ay};
	const float decel = _brake_decel_filter.update(-acceleration_xy.dot(velocity_xy_direction), _deltatime);

	if (PX4_ISFINITE(decel)) {
		_brake_trim = math::constrain(_brake_trim + _param_vt_b_dec_i * (decel_setpoint - decel) * _deltatime,
					      kBrakeTrimMin, kBrakeTrimMax);
	}

	const float demand = decel_setpoint + _brake_trim;

	// Dynamic pressure from calibrated airspeed (sea-level density in the law); ground speed without a sensor.
	float speed = velocity_xy.norm();
	const airspeed_validated_s &airspeed = _sub_airspeed.get();

	if (hrt_elapsed_time(&airspeed.timestamp) < 1_s && PX4_ISFINITE(airspeed.calibrated_airspeed_m_s)
	    && airspeed.airspeed_source != airspeed_validated_s::SOURCE_DISABLED) {
		speed = airspeed.calibrated_airspeed_m_s;
	}

	if (_brake_lift_observer) {
		// rotor vertical force from the MC thrust setpoint (normalised, MPC_THR_HOVER carries the weight)
		_sub_attitude.update();
		_sub_thrust_mc.update();
		const matrix::Quatf q_att{_sub_attitude.get().q};
		const float pitch_actual = matrix::Eulerf(q_att).theta();
		const float weight = _brake_params.mass * CONSTANTS_ONE_G;
		const float rotor_fz = -_sub_thrust_mc.get().xyz[2] / _hover_thrust * weight * cosf(pitch_actual);
		const float accel_up = -_sub_vehicle_local_position.get().az;
		_brake_params.lift_scale = vtol_brake::observeLiftScale(_brake_params, speed, _velocity(2), pitch_actual,
					   _brake_spoiler, accel_up, rotor_fz, _brake_params.lift_scale, _deltatime);
	}

	const float pitch_ideal = vtol_brake::idealPitch(_brake_params, speed, _velocity(2), demand);

	if (!PX4_ISFINITE(_brake_pitch)) {
		_brake_pitch = pitch_ideal;
	}

	const float step = _brake_pitch_rate * _deltatime;
	_brake_pitch += math::constrain(pitch_ideal - _brake_pitch, -step, step);

	const vtol_brake::Output out = vtol_brake::brake(_brake_params, speed, _velocity(2), demand, _brake_pitch);
	_brake_lift = out.lift;
	_brake_spoiler = out.spoiler;
	_brake_sink_max = vtol_brake::sinkMax(_brake_params, speed, demand);

	vtol_brake_setpoint_s setpoint{};
	setpoint.decel_setpoint = demand;
	setpoint.decel = decel;
	setpoint.pitch = _brake_pitch;
	setpoint.pitch_ideal = pitch_ideal;
	setpoint.spoiler = out.spoiler;
	setpoint.lift = out.lift;
	setpoint.sink_max = _brake_sink_max;
	setpoint.airspeed = speed;
	setpoint.lift_scale = _brake_params.lift_scale;
	setpoint.timestamp = hrt_absolute_time();
	_brake_setpoint_pub.publish(setpoint);

	return _brake_pitch;
}
