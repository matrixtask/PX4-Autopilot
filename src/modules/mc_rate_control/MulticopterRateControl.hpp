/****************************************************************************
 *
 *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
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

#pragma once

#include <lib/rate_control/rate_control.hpp>
#include <lib/mathlib/math/filter/AlphaFilter.hpp>
#include <lib/matrix/matrix/math.hpp>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <lib/systemlib/mavlink_log.h>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/actuator_controls_status.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/control_allocator_status.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/rate_ctrl_status.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/takeoff_status.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/vehicle_torque_setpoint.h>
#include <uORB/topics/effectiveness_id_status.h>

using namespace time_literals;

class MulticopterRateControl : public ModuleBase, public ModuleParams, public px4::WorkItem
{
public:
	static Descriptor desc;

	MulticopterRateControl(bool vtol = false);
	~MulticopterRateControl() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	void Run() override;

	/**
	 * initialize some vectors/matrices from parameters
	 */
	void parameters_updated();

	void updateActuatorControlsStatus(const vehicle_torque_setpoint_s &vehicle_torque_setpoint, float dt);

	RateControl _rate_control; ///< class for rate control calculations

	uORB::Subscription _battery_status_sub{ORB_ID(battery_status)};
	uORB::Subscription _control_allocator_status_sub{ORB_ID(control_allocator_status)};
	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};
	uORB::Subscription _takeoff_status_sub{ORB_ID(takeoff_status)};
	bool _standing{false};
	bool _rampup_after_stand{false};	///< takeoff ramp following the stand phase
	matrix::Vector2f _stand_integral{};	///< roll/pitch rate integral at the end of the stand phase
	float _stand_thrust{0.f};		///< collective thrust at the end of the stand phase	///< takeoff stand phase: on the main gear, attitude loop active, integrator learning
	uORB::Subscription _vehicle_rates_setpoint_sub{ORB_ID(vehicle_rates_setpoint)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::SubscriptionCallbackWorkItem _vehicle_angular_velocity_sub{this, ORB_ID(vehicle_angular_velocity)};

	uORB::Publication<actuator_controls_status_s>	_actuator_controls_status_pub{ORB_ID(actuator_controls_status_0)};
	uORB::PublicationMulti<rate_ctrl_status_s>	_controller_status_pub{ORB_ID(rate_ctrl_status)};
	uORB::Publication<vehicle_rates_setpoint_s>	_vehicle_rates_setpoint_pub{ORB_ID(vehicle_rates_setpoint)};
	uORB::Publication<vehicle_thrust_setpoint_s>	_vehicle_thrust_setpoint_pub;
	uORB::Publication<vehicle_torque_setpoint_s>	_vehicle_torque_setpoint_pub;

	vehicle_control_mode_s	_vehicle_control_mode{};
	vehicle_status_s	_vehicle_status{};

	bool _landed{true};
	bool _maybe_landed{true};

	hrt_abstime _last_run{0};

	perf_counter_t	_loop_perf;			/**< loop duration performance counter */

	// keep setpoint values between updates
	matrix::Vector3f _acro_rate_max;		/**< max attitude rates in acro mode */
	matrix::Vector3f _rates_setpoint{};

	float _battery_status_scale{0.0f};
	matrix::Vector3f _thrust_setpoint{};

	float _energy_integration_time{0.0f};
	float _control_energy[4] {};

	AlphaFilter<float> _output_lpf_yaw;

	// Angular-acceleration feed-forward (MC_ACC_FF_EN): torque for the acceleration the rate setpoint asks for,
	// 1 / MC_ID_B_x per axis, so a large inertia starts and brakes on the planned profile instead of after the
	// error has built up
	AlphaFilter<matrix::Vector3f> _rate_sp_dot_lpf;
	matrix::Vector3f _rate_sp_prev{NAN, NAN, NAN};

	// In-flight effectiveness identification (MC_ID_EN): per-axis scalar RLS with exponential forgetting on
	// angular acceleration = b * (torque through the actuator lag), both band-passed (0.2..3 Hz). The estimate is
	// saved provisionally on landing (MC_ID_PB_x); MC_ID_CONFIRM copies it to MC_ID_B_x and stops the estimation.
	void updateIdentification(const matrix::Vector3f &torque, const matrix::Vector3f &accel, float dt, bool in_air,
				  const matrix::Vector3f &rates, const matrix::Vector3f &rate_integral);
	// hover trim torque (roll, pitch): confirmed MC_TQ_TRIM_R/P + rate integral, averaged while quiet
	float _id_trim[2] {};
	float _id_trim_quiet_s{0.f};
	void confirmIdentification();
	matrix::Vector3f _id_tq_lag{};
	AlphaFilter<matrix::Vector3f> _id_u_fast, _id_u_slow, _id_y_fast, _id_y_slow;
	float _id_b[3] {NAN, NAN, NAN};
	float _id_p[3] {};
	float _id_excitation_s[3] {};
	float _id_excitation_max[3] {};
	float _id_sat_hold{0.f};
	bool _id_was_in_air{false};
	hrt_abstime _id_last_pub{0};
	orb_advert_t _mavlink_log_pub{nullptr};
	param_t _param_mpc_thr_hover_id{param_find("MPC_THR_HOVER")};
	float _thr_hover{0.5f};
	uORB::Publication<effectiveness_id_status_s> _effectiveness_id_status_pub{ORB_ID(effectiveness_id_status)};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::MC_ROLLRATE_P>) _param_mc_rollrate_p,
		(ParamFloat<px4::params::MC_ROLLRATE_I>) _param_mc_rollrate_i,
		(ParamFloat<px4::params::MC_RR_INT_LIM>) _param_mc_rr_int_lim,
		(ParamFloat<px4::params::MC_ROLLRATE_D>) _param_mc_rollrate_d,
		(ParamFloat<px4::params::MC_ROLLRATE_FF>) _param_mc_rollrate_ff,
		(ParamFloat<px4::params::MC_ROLLRATE_K>) _param_mc_rollrate_k,

		(ParamFloat<px4::params::MC_PITCHRATE_P>) _param_mc_pitchrate_p,
		(ParamFloat<px4::params::MC_PITCHRATE_I>) _param_mc_pitchrate_i,
		(ParamFloat<px4::params::MC_PR_INT_LIM>) _param_mc_pr_int_lim,
		(ParamFloat<px4::params::MC_PITCHRATE_D>) _param_mc_pitchrate_d,
		(ParamFloat<px4::params::MC_PITCHRATE_FF>) _param_mc_pitchrate_ff,
		(ParamFloat<px4::params::MC_PITCHRATE_K>) _param_mc_pitchrate_k,

		(ParamFloat<px4::params::MC_YAWRATE_P>) _param_mc_yawrate_p,
		(ParamFloat<px4::params::MC_YAWRATE_I>) _param_mc_yawrate_i,
		(ParamFloat<px4::params::MC_YR_INT_LIM>) _param_mc_yr_int_lim,
		(ParamFloat<px4::params::MC_YAWRATE_D>) _param_mc_yawrate_d,
		(ParamFloat<px4::params::MC_YAWRATE_FF>) _param_mc_yawrate_ff,
		(ParamFloat<px4::params::MC_YAWRATE_K>) _param_mc_yawrate_k,
		(ParamFloat<px4::params::MC_YAW_TQ_CUTOFF>) _param_mc_yaw_tq_cutoff,
		(ParamInt<px4::params::MC_ACC_FF_EN>) _param_mc_acc_ff_en,
		(ParamInt<px4::params::MC_ID_EN>) _param_mc_id_en,
		(ParamInt<px4::params::MC_ID_CONFIRM>) _param_mc_id_confirm,
		(ParamFloat<px4::params::MC_ID_T>) _param_mc_id_t,
		(ParamFloat<px4::params::MC_ID_TAU>) _param_mc_id_tau,
		(ParamFloat<px4::params::MC_ID_EXC>) _param_mc_id_exc,
		(ParamFloat<px4::params::MC_ID_TQLIM>) _param_mc_id_tqlim,
		(ParamFloat<px4::params::MC_ID_B_R>) _param_mc_id_b_r,
		(ParamFloat<px4::params::MC_ID_B_P>) _param_mc_id_b_p,
		(ParamFloat<px4::params::MC_ID_B_Y>) _param_mc_id_b_y,
		(ParamFloat<px4::params::MC_ID_PB_R>) _param_mc_id_pb_r,
		(ParamFloat<px4::params::MC_ID_PB_P>) _param_mc_id_pb_p,
		(ParamFloat<px4::params::MC_ID_PB_Y>) _param_mc_id_pb_y,
		(ParamFloat<px4::params::MC_ID_PX_R>) _param_mc_id_px_r,
		(ParamFloat<px4::params::MC_ID_PX_P>) _param_mc_id_px_p,
		(ParamFloat<px4::params::MC_ID_PX_Y>) _param_mc_id_px_y,
		(ParamFloat<px4::params::MC_ID_PT_R>) _param_mc_id_pt_r,
		(ParamFloat<px4::params::MC_ID_PT_P>) _param_mc_id_pt_p,
		(ParamFloat<px4::params::MC_TQ_TRIM_R>) _param_mc_tq_trim_r,
		(ParamFloat<px4::params::MC_TQ_TRIM_P>) _param_mc_tq_trim_p,
		(ParamFloat<px4::params::MC_TQ_STD_P>) _param_mc_tq_std_p,

		(ParamFloat<px4::params::MC_ACRO_R_MAX>) _param_mc_acro_r_max,
		(ParamFloat<px4::params::MC_ACRO_P_MAX>) _param_mc_acro_p_max,
		(ParamFloat<px4::params::MC_ACRO_Y_MAX>) _param_mc_acro_y_max,
		(ParamFloat<px4::params::MC_ACRO_EXPO>) _param_mc_acro_expo,			/**< expo stick curve shape (roll & pitch) */
		(ParamFloat<px4::params::MC_ACRO_EXPO_Y>) _param_mc_acro_expo_y,				/**< expo stick curve shape (yaw) */
		(ParamFloat<px4::params::MC_ACRO_SUPEXPO>) _param_mc_acro_supexpo,		/**< superexpo stick curve shape (roll & pitch) */
		(ParamFloat<px4::params::MC_ACRO_SUPEXPOY>) _param_mc_acro_supexpoy,		/**< superexpo stick curve shape (yaw) */

		(ParamBool<px4::params::MC_BAT_SCALE_EN>) _param_mc_bat_scale_en
	)
};
