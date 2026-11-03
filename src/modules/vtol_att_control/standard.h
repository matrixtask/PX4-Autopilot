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
* @file standard.h
* VTOL with fixed multirotor motor configurations (such as quad) and a pusher
* (or puller aka tractor) motor for forward flight.
*
* @author Simon Wilks 		<simon@uaventure.com>
* @author Roman Bapst 		<bapstroman@gmail.com>
* @author Andreas Antener	<andreas@uaventure.com>
* @author Sander Smeets 	<sander@droneslab.com>
*
*/

#ifndef STANDARD_H
#define STANDARD_H
#include "vtol_type.h"
#include <uORB/Subscription.hpp>
#include <uORB/topics/vtol_brake_setpoint.h>
#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/topics/control_allocator_status.h>
#include <uORB/topics/upset_recovery_status.h>

class Standard : public VtolType
{

public:

	Standard(VtolAttitudeControl *_att_controller);
	~Standard() override = default;

	void update_vtol_state() override;
	void update_transition_state() override;
	void update_fw_state() override;
	void update_mc_state() override;
	void fill_actuator_outputs() override;
	void waiting_on_tecs() override;
	void blendThrottleAfterFrontTransition(float scale) override;

private:

	enum class vtol_mode {
		MC_MODE = 0,
		TRANSITION_TO_FW,
		TRANSITION_TO_MC,
		FW_MODE
	};

	vtol_mode _vtol_mode{vtol_mode::MC_MODE};			/**< vtol flight mode, defined by enum vtol_mode */

	float _pusher_throttle{0.0f};
	float _airspeed_trans_blend_margin{0.0f};
	hrt_abstime _last_time_pusher_transition_update{0};
	// Mk-7 smooth lift->wing handoff: fade the lift-rotor attitude+thrust over ~2 s after FW entry instead of
	// an abrupt cut (the abrupt stop momentarily exceeded the small v6 ailerons' roll authority -> roll-over).
	hrt_abstime _fw_mode_entry_time{0};
	float _front_transition_end_thrust{0.f}; ///< lift-rotor collective (body z, up < 0) at the end of the front transition

	// Airspeed schedule (Veronte-style sigma): every rotor/wing schedule is a function of the filtered airspeed, and the
	// rotor share changes at a bounded rate, so a phase change never steps an actuator group.
	float updateScheduleAirspeed(); ///< first-order filtered CAS (VT_SIG_TAU), NAN without airspeed
	float _cas_schedule{NAN};
	hrt_abstime _cas_schedule_time{0};
	float _ft_mc_weight{1.f};       ///< rate-limited MC weight in the front transition (VT_SIG_RATE)
	hrt_abstime _ft_weight_time{0};

	// Rotor -> wing handover after the front transition: the lift-rotor collective slews (VT_LIFT_HND_SLW) from its
	// value at the end of the transition to the airspeed schedule and then to zero, with a climb-rate assist
	// (VT_LIFT_HND_KV) while the wing takes the load.
	float liftHandoverCollective(float cas);
	float _handover_collective{0.f}; ///< lift-rotor up-thrust [0, 1] in FW
	hrt_abstime _handover_time{0};

	// CCV feed-forward (VT_LIFT_HND_AOA): pitch for the lift the rotors give up after the front transition
	static constexpr float kCcvWashoutTimeConstant = 20.f; ///< [s] wash-out once the rotors are off (TECS takes over)
	float _ccv_share_start{0.f};  ///< rotor weight share at FW entry
	float _ccv_pitch{0.f};        ///< [rad] pitch feed-forward
	hrt_abstime _ccv_washout_time{0};

	// Feed-forward back-transition brake (VT_BFF_EN, FlightTaskTransition): pitch comes with the MC attitude setpoint
	uORB::Subscription _brake_setpoint_sub{ORB_ID(vtol_brake_setpoint)};
	vtol_brake_setpoint_s _brake_setpoint{};
	bool brakeFeedForwardActive();
	bool _mc_throttle_weight_override{false}; ///< full collective in the feed-forward back transition

	void parameters_update() override;

	// Torque redistribution between the two actuator groups (VT_RD_EN). Each group's torque setpoint is normalised to
	// its own authority: surfaces to KS q (dynamic pressure from CAS), lift rotors to BU (rho / rho0). A torque one
	// group cannot give (unallocated, from the control allocator) is converted through that ratio and added to the
	// other group, so the total moment follows the demand until both are saturated. In fixed-wing flight the lift
	// rotors are spun up to the collective the differential needs and kept for VT_RD_HOLD after the last excess.
	void redistributeTorque();
	uORB::SubscriptionMultiArray<control_allocator_status_s, 2> _ca_status_subs{ORB_ID::control_allocator_status};
	control_allocator_status_s _ca_status[2] {};
	uORB::Subscription _upset_status_sub{ORB_ID(upset_recovery_status)};
	float _rd_collective{0.f};	///< lift-rotor up-thrust for the redistributed differential [0, 1]
	float _rd_pitch{0.f};		///< [rad] pitch feed-forward for the lift of that collective
	hrt_abstime _rd_last_excess{0};
	hrt_abstime _rd_time{0};

	DEFINE_PARAMETERS_CUSTOM_PARENT(VtolType,
					(ParamFloat<px4::params::VT_PSHER_SLEW>) _param_vt_psher_slew,
					(ParamFloat<px4::params::VT_B_TRANS_RAMP>) _param_vt_b_trans_ramp,
					(ParamFloat<px4::params::VT_B_RAMP_MIN>) _param_vt_b_ramp_min,
					(ParamFloat<px4::params::VT_LIFT_HND_V>) _param_vt_lift_hnd_v,
					(ParamFloat<px4::params::VT_LIFT_HND_THR>) _param_vt_lift_hnd_thr,
					(ParamFloat<px4::params::VT_LIFT_HND_SLW>) _param_vt_lift_hnd_slw,
					(ParamFloat<px4::params::VT_LIFT_HND_KV>) _param_vt_lift_hnd_kv,
					(ParamFloat<px4::params::VT_LIFT_HND_AOA>) _param_vt_lift_hnd_aoa,
					(ParamFloat<px4::params::VT_SIG_TAU>) _param_vt_sig_tau,
					(ParamFloat<px4::params::VT_SIG_RATE>) _param_vt_sig_rate,
					(ParamFloat<px4::params::VT_FW_RDIFF>) _param_vt_fw_rdiff,
					(ParamFloat<px4::params::VT_FW_MC_THR>) _param_vt_fw_mc_thr,
					(ParamBool<px4::params::VT_BFF_EN>) _param_vt_bff_en,
					(ParamFloat<px4::params::VT_BFF_VEXIT>) _param_vt_bff_vexit,
					(ParamFloat<px4::params::FW_PSP_OFF>) _param_fw_psp_off,
					(ParamBool<px4::params::VT_RD_EN>) _param_vt_rd_en,
					(ParamFloat<px4::params::VT_RD_KS_R>) _param_vt_rd_ks_r,
					(ParamFloat<px4::params::VT_RD_KS_P>) _param_vt_rd_ks_p,
					(ParamFloat<px4::params::VT_RD_BU_R>) _param_vt_rd_bu_r,
					(ParamFloat<px4::params::VT_RD_BU_P>) _param_vt_rd_bu_p,
					(ParamFloat<px4::params::VT_RD_COLL>) _param_vt_rd_coll,
					(ParamFloat<px4::params::VT_RD_CMAX>) _param_vt_rd_cmax,
					(ParamFloat<px4::params::VT_RD_HOLD>) _param_vt_rd_hold,
					(ParamFloat<px4::params::VT_RD_SLEW>) _param_vt_rd_slew,
					(ParamFloat<px4::params::VT_RD_VMIN>) _param_vt_rd_vmin
				       )
};
#endif
