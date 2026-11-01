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
 * @file FlightTaskTransition.hpp
 *
 * Flight task for automatic VTOL transitions between hover and forward flight and vice versa.
 */

#pragma once

#include "FlightTask.hpp"
#include <lib/mathlib/math/filter/AlphaFilter.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/airspeed_validated.h>
#include <uORB/topics/vtol_brake_setpoint.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/Publication.hpp>
#include <drivers/drv_hrt.h>
#include <lib/vtol_brake/BackTransitionBrake.hpp>

using namespace time_literals;


class FlightTaskTransition : public FlightTask
{
public:
	FlightTaskTransition();

	virtual ~FlightTaskTransition() = default;
	bool activate(const trajectory_setpoint_s &last_setpoint) override;
	bool updateInitialize() override;
	bool update() override;

private:
	static constexpr float kVerticalVelocityTimeConstant = 2.0f;
	static constexpr float kDecelerationIntegratorLimit = 0.3f;
	static constexpr float kAccelerationFilterTimeConstant = 0.05f;

	uORB::SubscriptionData<vehicle_status_s> _sub_vehicle_status{ORB_ID(vehicle_status)};
	uORB::SubscriptionData<position_setpoint_triplet_s> _sub_position_sp_triplet{ORB_ID(position_setpoint_triplet)};
	uORB::SubscriptionData<airspeed_validated_s> _sub_airspeed{ORB_ID(airspeed_validated)};
	uORB::Publication<vtol_brake_setpoint_s> _brake_setpoint_pub{ORB_ID(vtol_brake_setpoint)};
	uORB::SubscriptionData<vehicle_attitude_s> _sub_attitude{ORB_ID(vehicle_attitude)};
	uORB::SubscriptionData<vehicle_thrust_setpoint_s> _sub_thrust_mc{ORB_ID(vehicle_thrust_setpoint), 0};

	float _param_fw_psp_off{0.f};
	float _param_vt_b_dec_i{0.f};
	float _param_vt_b_dec_mss{0.f};
	float _param_vt_f_trans_vz{0.f};    ///< climb rate during front transition [m/s], 0 = hold altitude (upstream)
	float _param_vt_b_trans_vz{0.f};    ///< descent rate during back transition at speed [m/s], 0 = hold altitude
	float _param_vt_b_trans_vzlo{0.f};  ///< descent rate limit at low speed (vortex ring state) [m/s]
	float _param_vt_b_trans_hmin{0.f};  ///< lowest height above the back-transition target [m]
	int32_t _param_vt_f_yaw_hold{0};     ///< hold the start heading during the front transition
	float _transition_yaw{NAN};          ///< heading at the start of the front transition [rad]

	// Feed-forward back-transition brake (VT_BFF_*, lib/vtol_brake)
	static constexpr float kLateralGuidanceGain = 3.f;      ///< proportional-navigation constant N
	static constexpr float kLateralGuidanceMaxAccel = 2.f;  ///< [m/s^2] ~12 deg bank
	static constexpr float kLateralGuidanceMinDist = 15.f;  ///< [m] below: MC position control
	static constexpr float kBrakeDecelCap = 1.25f;   ///< decel demand <= 1.25 VT_B_DEC_MSS (bt_sim: 2x saturates the brake)
	static constexpr float kBrakeJerk = 1.5f;        ///< [m/s^3] ramp-in of the decel demand (comfort)
	static constexpr float kBrakeTrimMin = -0.5f;    ///< [m/s^2] limits of the decel trim integrator
	static constexpr float kBrakeTrimMax = 1.5f;
	static constexpr float kBrakeDecelFilterTimeConstant = 0.3f;
	bool _brake_ff_enabled{false};
	vtol_brake::Params _brake_params{};
	float _brake_pitch_rate{0.f};        ///< [rad/s] pitch-setpoint rate limit
	int32_t _param_mpc_acc_decouple{1};
	float _param_mpc_jerk_auto{0.f};      ///< MC trajectory jerk: braking taper at the end of the back transition
	float _param_mpc_acc_hor{0.f};        ///< MC trajectory acceleration the braking hands over to
	float _param_vt_bff_vexit{0.f};       ///< speed at which the back transition hands over to MC
	float _brake_pitch{NAN};             ///< [rad] rate-limited pitch command
	float _brake_trim{0.f};              ///< [m/s^2] decel trim (VT_B_DEC_I) on top of the feed-forward
	float _brake_lift{0.f};              ///< [N] predicted wing lift
	float _brake_sink_max{INFINITY};     ///< [m/s] sink that still leaves the angle of attack to brake
	float _brake_time{0.f};              ///< [s] since the back transition started
	bool _brake_lift_observer{false};    ///< VT_BFF_LOBS
	float _hover_thrust{0.5f};           ///< MPC_THR_HOVER: normalised thrust that carries the weight
	float _brake_spoiler{0.f};           ///< last spoiler command (for the lift observer)
	AlphaFilter<float> _brake_decel_filter{kBrakeDecelFilterTimeConstant};

	static constexpr float kAltitudeCaptureTimeConstant = 5.0f; ///< [s] climb/descent rate fades out near the target
	static constexpr float kLowSpeedSink = 15.4f;   ///< [m/s] (30 kt) below this only VT_B_TRANS_VZLO is allowed
	static constexpr float kFullSpeedSink = 25.7f;  ///< [m/s] (50 kt) above this the full VT_B_TRANS_VZ is allowed
	static constexpr float kFullSinkMaxSpeed = 30.9f; ///< [m/s] (60 kt) full sink up to this speed ...
	static constexpr float kNoSinkSpeed = 41.2f;      ///< [m/s] (80 kt) ... and none above: brake first, then descend

	AlphaFilter<float> _vel_z_filter{kVerticalVelocityTimeConstant};
	AlphaFilter<matrix::Vector2f> _accel_filter{kAccelerationFilterTimeConstant};
	float _decel_error_bt_int{0.f}; ///< Backtransition deceleration error integrator value

	float computeBackTransitionTiltSetpoint();
	float computeBackTransitionDecelSetpoint() const;
	bool backTransitionTarget(matrix::Vector2f &pos_to_target) const;
	matrix::Vector2f computeBackTransitionLateralAccel() const;
	float computeBackTransitionBrake();
	void loadBrakeParams();
	float computeVerticalVelocityTarget() const;
};
