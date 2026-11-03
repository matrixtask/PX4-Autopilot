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

#include "MulticopterRateControl.hpp"

#include <drivers/drv_hrt.h>
#include <circuit_breaker/circuit_breaker.h>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>
#include <px4_platform_common/events.h>

using namespace matrix;
using namespace time_literals;
using math::radians;

ModuleBase::Descriptor MulticopterRateControl::desc{task_spawn, custom_command, print_usage};

MulticopterRateControl::MulticopterRateControl(bool vtol) :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),
	_vehicle_thrust_setpoint_pub(vtol ? ORB_ID(vehicle_thrust_setpoint_virtual_mc) : ORB_ID(vehicle_thrust_setpoint)),
	_vehicle_torque_setpoint_pub(vtol ? ORB_ID(vehicle_torque_setpoint_virtual_mc) : ORB_ID(vehicle_torque_setpoint)),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	_vehicle_status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;

	parameters_updated();
	_controller_status_pub.advertise();
}

MulticopterRateControl::~MulticopterRateControl()
{
	perf_free(_loop_perf);
}

bool
MulticopterRateControl::init()
{
	if (!_vehicle_angular_velocity_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

void
MulticopterRateControl::parameters_updated()
{
	// rate control parameters
	// The controller gain K is used to convert the parallel (P + I/s + sD) form
	// to the ideal (K * [1 + 1/sTi + sTd]) form
	const Vector3f rate_k = Vector3f(_param_mc_rollrate_k.get(), _param_mc_pitchrate_k.get(), _param_mc_yawrate_k.get());

	_rate_control.setPidGains(
		rate_k.emult(Vector3f(_param_mc_rollrate_p.get(), _param_mc_pitchrate_p.get(), _param_mc_yawrate_p.get())),
		rate_k.emult(Vector3f(_param_mc_rollrate_i.get(), _param_mc_pitchrate_i.get(), _param_mc_yawrate_i.get())),
		rate_k.emult(Vector3f(_param_mc_rollrate_d.get(), _param_mc_pitchrate_d.get(), _param_mc_yawrate_d.get())));

	_rate_control.setIntegratorLimit(
		Vector3f(_param_mc_rr_int_lim.get(), _param_mc_pr_int_lim.get(), _param_mc_yr_int_lim.get()));

	_rate_control.setFeedForwardGain(
		Vector3f(_param_mc_rollrate_ff.get(), _param_mc_pitchrate_ff.get(), _param_mc_yawrate_ff.get()));


	// manual rate control acro mode rate limits
	_acro_rate_max = Vector3f(radians(_param_mc_acro_r_max.get()), radians(_param_mc_acro_p_max.get()),
				  radians(_param_mc_acro_y_max.get()));

	_output_lpf_yaw.setCutoffFreq(_param_mc_yaw_tq_cutoff.get());
	_rate_sp_dot_lpf.setCutoffFreq(2.f);
	_id_u_fast.setCutoffFreq(3.f);
	_id_y_fast.setCutoffFreq(3.f);
	_id_u_slow.setCutoffFreq(0.2f);
	_id_y_slow.setCutoffFreq(0.2f);

	// a human confirmed the provisional estimate (QGC / param set MC_ID_CONFIRM 1), only applied on the ground
	if (_param_mc_id_confirm.get() == 1 && !_vehicle_control_mode.flag_armed) {
		confirmIdentification();
	}
}

void
MulticopterRateControl::Run()
{
	if (should_exit()) {
		_vehicle_angular_velocity_sub.unregisterCallback();
		exit_and_cleanup(desc);
		return;
	}

	perf_begin(_loop_perf);

	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		updateParams();
		parameters_updated();
	}

	/* run controller on gyro changes */
	vehicle_angular_velocity_s angular_velocity;

	if (_vehicle_angular_velocity_sub.update(&angular_velocity)) {

		const hrt_abstime now = angular_velocity.timestamp_sample;

		// Guard against too small (< 0.125ms) and too large (> 20ms) dt's.
		const float dt = math::constrain(((now - _last_run) * 1e-6f), 0.000125f, 0.02f);
		_last_run = now;

		const Vector3f rates{angular_velocity.xyz};
		const Vector3f angular_accel{angular_velocity.xyz_derivative};

		/* check for updates in other topics */
		_vehicle_control_mode_sub.update(&_vehicle_control_mode);

		if (_vehicle_land_detected_sub.updated()) {
			vehicle_land_detected_s vehicle_land_detected;

			if (_vehicle_land_detected_sub.copy(&vehicle_land_detected)) {
				_landed = vehicle_land_detected.landed;
				_maybe_landed = vehicle_land_detected.maybe_landed;
			}
		}

		_vehicle_status_sub.update(&_vehicle_status);

		// use rates setpoint topic
		vehicle_rates_setpoint_s vehicle_rates_setpoint{};

		if (_vehicle_control_mode.flag_control_manual_enabled && !_vehicle_control_mode.flag_control_attitude_enabled) {
			// generate the rate setpoint from sticks
			manual_control_setpoint_s manual_control_setpoint;

			if (_manual_control_setpoint_sub.update(&manual_control_setpoint)) {
				// manual rates control - ACRO mode
				const Vector3f man_rate_sp{
					math::superexpo(manual_control_setpoint.roll, _param_mc_acro_expo.get(), _param_mc_acro_supexpo.get()),
					math::superexpo(-manual_control_setpoint.pitch, _param_mc_acro_expo.get(), _param_mc_acro_supexpo.get()),
					math::superexpo(manual_control_setpoint.yaw, _param_mc_acro_expo_y.get(), _param_mc_acro_supexpoy.get())};

				_rates_setpoint = man_rate_sp.emult(_acro_rate_max);
				_thrust_setpoint(2) = -(manual_control_setpoint.throttle + 1.f) * .5f;
				_thrust_setpoint(0) = _thrust_setpoint(1) = 0.f;

				// publish rate setpoint
				vehicle_rates_setpoint.roll = _rates_setpoint(0);
				vehicle_rates_setpoint.pitch = _rates_setpoint(1);
				vehicle_rates_setpoint.yaw = _rates_setpoint(2);
				_thrust_setpoint.copyTo(vehicle_rates_setpoint.thrust_body);
				vehicle_rates_setpoint.timestamp = hrt_absolute_time();

				_vehicle_rates_setpoint_pub.publish(vehicle_rates_setpoint);
			}

		} else if (_vehicle_rates_setpoint_sub.update(&vehicle_rates_setpoint)) {
			_rates_setpoint(0) = PX4_ISFINITE(vehicle_rates_setpoint.roll)  ? vehicle_rates_setpoint.roll  : rates(0);
			_rates_setpoint(1) = PX4_ISFINITE(vehicle_rates_setpoint.pitch) ? vehicle_rates_setpoint.pitch : rates(1);
			_rates_setpoint(2) = PX4_ISFINITE(vehicle_rates_setpoint.yaw)   ? vehicle_rates_setpoint.yaw   : rates(2);
			_thrust_setpoint = Vector3f(vehicle_rates_setpoint.thrust_body);
		}

		// run the rate controller
		if (_vehicle_control_mode.flag_control_rates_enabled) {

			// reset integral if disarmed
			if (!_vehicle_control_mode.flag_armed || _vehicle_status.vehicle_type != vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
				_rate_control.resetIntegral();
			}

			// update saturation status from control allocation feedback
			control_allocator_status_s control_allocator_status;

			if (_control_allocator_status_sub.update(&control_allocator_status)) {
				Vector<bool, 3> saturation_positive;
				Vector<bool, 3> saturation_negative;

				if (!control_allocator_status.torque_setpoint_achieved) {
					for (size_t i = 0; i < 3; i++) {
						if (control_allocator_status.unallocated_torque[i] > FLT_EPSILON) {
							saturation_positive(i) = true;

						} else if (control_allocator_status.unallocated_torque[i] < -FLT_EPSILON) {
							saturation_negative(i) = true;
						}
					}
				}

				// TODO: send the unallocated value directly for better anti-windup
				_rate_control.setSaturationStatus(saturation_positive, saturation_negative);
			}

			// run rate controller
			Vector3f torque_setpoint =
				_rate_control.update(rates, _rates_setpoint, angular_accel, dt, _maybe_landed || _landed);

			// hover trim torque (roll/pitch): the steady moment of the CG offset. Applied from arming, so it is
			// there at lift-off; the rate integrator is held at zero on the ground and took >10 s to build it
			// (Mk-7 SITL: 0.016 pitch, the nose sagged 4-6 deg for seconds after lift-off and the aircraft drifted)
			if (_vehicle_control_mode.flag_armed) {
				torque_setpoint(0) += _param_mc_tq_trim_r.get();
				torque_setpoint(1) += _param_mc_tq_trim_p.get();
			}

			// angular-acceleration feed-forward: d/dt of the rate setpoint (2 Hz low-pass) / MC_ID_B_x, per axis
			const bool airborne = _vehicle_control_mode.flag_armed && !_landed && !_maybe_landed;
			const float b_conf[3] {_param_mc_id_b_r.get(), _param_mc_id_b_p.get(), _param_mc_id_b_y.get()};

			if (_param_mc_acc_ff_en.get() != 0 && airborne && dt > 1e-4f && _rate_sp_prev.isAllFinite()) {
				const Vector3f acc_sp = _rate_sp_dot_lpf.update((_rates_setpoint - _rate_sp_prev) / dt, dt);

				for (int i = 0; i < 3; i++) {
					if ((_param_mc_acc_ff_en.get() & (1 << i)) && b_conf[i] > 0.02f) {
						// bounded: a rate setpoint step (mode switch) must not kick the axis
						torque_setpoint(i) += math::constrain(acc_sp(i) / b_conf[i], -0.3f, 0.3f);
					}
				}

			} else {
				_rate_sp_dot_lpf.reset(Vector3f{});
			}

			_rate_sp_prev = _rates_setpoint;

			// apply low-pass filtering on yaw axis to reduce high frequency torque caused by rotor acceleration
			torque_setpoint(2) = _output_lpf_yaw.update(torque_setpoint(2), dt);

			{
				rate_ctrl_status_s integ{};
				_rate_control.getRateControlStatus(integ);
				updateIdentification(torque_setpoint, angular_accel, dt,
						     airborne && _vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING,
						     rates, Vector3f{integ.rollspeed_integ, integ.pitchspeed_integ, integ.yawspeed_integ});
			}

			// publish rate controller status
			rate_ctrl_status_s rate_ctrl_status{};
			_rate_control.getRateControlStatus(rate_ctrl_status);
			rate_ctrl_status.timestamp = hrt_absolute_time();
			_controller_status_pub.publish(rate_ctrl_status);

			// publish thrust and torque setpoints
			vehicle_thrust_setpoint_s vehicle_thrust_setpoint{};
			vehicle_torque_setpoint_s vehicle_torque_setpoint{};

			_thrust_setpoint.copyTo(vehicle_thrust_setpoint.xyz);
			vehicle_torque_setpoint.xyz[0] = PX4_ISFINITE(torque_setpoint(0)) ? torque_setpoint(0) : 0.f;
			vehicle_torque_setpoint.xyz[1] = PX4_ISFINITE(torque_setpoint(1)) ? torque_setpoint(1) : 0.f;
			vehicle_torque_setpoint.xyz[2] = PX4_ISFINITE(torque_setpoint(2)) ? torque_setpoint(2) : 0.f;

			// scale setpoints by battery status if enabled
			if (_param_mc_bat_scale_en.get()) {
				if (_battery_status_sub.updated()) {
					battery_status_s battery_status;

					if (_battery_status_sub.copy(&battery_status) && battery_status.connected && battery_status.scale > 0.f) {
						_battery_status_scale = battery_status.scale;
					}
				}

				if (_battery_status_scale > 0.f) {
					for (int i = 0; i < 3; i++) {
						vehicle_thrust_setpoint.xyz[i] = math::constrain(vehicle_thrust_setpoint.xyz[i] * _battery_status_scale, -1.f, 1.f);
						vehicle_torque_setpoint.xyz[i] = math::constrain(vehicle_torque_setpoint.xyz[i] * _battery_status_scale, -1.f, 1.f);
					}
				}
			}

			vehicle_thrust_setpoint.timestamp_sample = angular_velocity.timestamp_sample;
			vehicle_thrust_setpoint.timestamp = hrt_absolute_time();
			_vehicle_thrust_setpoint_pub.publish(vehicle_thrust_setpoint);

			vehicle_torque_setpoint.timestamp_sample = angular_velocity.timestamp_sample;
			vehicle_torque_setpoint.timestamp = hrt_absolute_time();
			_vehicle_torque_setpoint_pub.publish(vehicle_torque_setpoint);

			updateActuatorControlsStatus(vehicle_torque_setpoint, dt);

		}
	}

	perf_end(_loop_perf);
}

void MulticopterRateControl::confirmIdentification()
{
	// provisional -> confirmed for every axis that has a provisional value; then stop estimating and saving
	param_t pb[3] {param_find("MC_ID_PB_R"), param_find("MC_ID_PB_P"), param_find("MC_ID_PB_Y")};
	param_t px[3] {param_find("MC_ID_PX_R"), param_find("MC_ID_PX_P"), param_find("MC_ID_PX_Y")};
	param_t bc[3] {param_find("MC_ID_B_R"), param_find("MC_ID_B_P"), param_find("MC_ID_B_Y")};
	static constexpr char axis_name[3] {'R', 'P', 'Y'};
	const float zero = 0.f;

	for (int i = 0; i < 3; i++) {
		float v = 0.f;
		param_get(pb[i], &v);

		if (v > 0.f) {
			param_set_no_notification(bc[i], &v);
			PX4_INFO("MC_ID_B_%c confirmed: %.3f rad/s^2 per unit torque", axis_name[i], (double)v);
		}

		param_set_no_notification(pb[i], &zero);
		param_set_no_notification(px[i], &zero);
	}

	const param_t pt[2] {param_find("MC_ID_PT_R"), param_find("MC_ID_PT_P")};
	const param_t tt[2] {param_find("MC_TQ_TRIM_R"), param_find("MC_TQ_TRIM_P")};

	for (int i = 0; i < 2; i++) {
		float v = 0.f;
		param_get(pt[i], &v);

		if (fabsf(v) > FLT_EPSILON) {	// 0 = no provisional trim
			param_set_no_notification(tt[i], &v);
			PX4_INFO("MC_TQ_TRIM_%c confirmed: %.4f", axis_name[i], (double)v);
		}

		param_set_no_notification(pt[i], &zero);
	}

	const int32_t off = 0;
	param_set_no_notification(param_find("MC_ID_EN"), &off);
	param_set(param_find("MC_ID_CONFIRM"), &off);	// the last one notifies: QGC and this module reload
	mavlink_log_info(&_mavlink_log_pub, "Control effectiveness confirmed, identification stopped\t");

	for (int i = 0; i < 3; i++) {
		_id_b[i] = NAN;
	}
}

void MulticopterRateControl::updateIdentification(const Vector3f &torque, const Vector3f &accel, float dt,
		bool in_air, const Vector3f &rates, const Vector3f &rate_integral)
{
	if (_param_mc_id_en.get() != 1 || !torque.isAllFinite() || !accel.isAllFinite()) {
		_id_was_in_air = false;
		return;
	}

	static constexpr float P0 = 100.f;	// initial / maximum covariance
	static constexpr float kMinExcitation = 5.f;	// [s] of usable excitation before an estimate is trusted
	static constexpr char axis_name[3] {'R', 'P', 'Y'};
	const float pb[3] {_param_mc_id_pb_r.get(), _param_mc_id_pb_p.get(), _param_mc_id_pb_y.get()};
	const float bc[3] {_param_mc_id_b_r.get(), _param_mc_id_b_p.get(), _param_mc_id_b_y.get()};

	for (int i = 0; i < 3; i++) {
		if (!PX4_ISFINITE(_id_b[i])) {	// start from the provisional value, else the confirmed one
			_id_b[i] = pb[i] > 0.02f ? pb[i] : (bc[i] > 0.02f ? bc[i] : 1.f);
			_id_p[i] = P0;
		}
	}

	// on landing: save the estimate of every axis that converged in this flight as provisional (MC_ID_PB_x) with its
	// excitation (MC_ID_PX_x) for review. Confirmation is a human decision (MC_ID_CONFIRM).
	if (_id_was_in_air && !in_air) {
		const param_t pbh[3] {param_find("MC_ID_PB_R"), param_find("MC_ID_PB_P"), param_find("MC_ID_PB_Y")};
		const param_t pxh[3] {param_find("MC_ID_PX_R"), param_find("MC_ID_PX_P"), param_find("MC_ID_PX_Y")};

		for (int i = 0; i < 3; i++) {
			if (_id_excitation_max[i] >= kMinExcitation) {
				param_set_no_notification(pbh[i], &_id_b[i]);
				param_set_no_notification(pxh[i], &_id_excitation_max[i]);
				PX4_INFO("MC_ID_PB_%c provisional: %.3f rad/s^2 per unit torque (%.0f s excitation)", axis_name[i],
					 (double)_id_b[i], (double)_id_excitation_max[i]);
			}
		}

		if (_id_trim_quiet_s >= 10.f) {	// hover trim from at least 10 s of quiet hover
			const param_t pth[2] {param_find("MC_ID_PT_R"), param_find("MC_ID_PT_P")};

			for (int i = 0; i < 2; i++) {
				param_set_no_notification(pth[i], &_id_trim[i]);
			}

			PX4_INFO("MC_ID_PT provisional hover trim torque: roll %.4f pitch %.4f (%.0f s quiet hover)",
				 (double)_id_trim[0], (double)_id_trim[1], (double)_id_trim_quiet_s);
		}
	}

	if (!_id_was_in_air && in_air) {
		for (int i = 0; i < 3; i++) { _id_excitation_max[i] = 0.f; }

		_id_trim_quiet_s = 0.f;
		_id_trim[0] = _param_mc_tq_trim_r.get();
		_id_trim[1] = _param_mc_tq_trim_p.get();
	}

	_id_was_in_air = in_air;

	if (!in_air) {
		_id_tq_lag = torque;
		_id_u_fast.reset(Vector3f{});
		_id_u_slow.reset(Vector3f{});
		_id_y_fast.reset(Vector3f{});
		_id_y_slow.reset(Vector3f{});
		return;
	}

	// torque through the actuator lag (rotor speed time constant), then the same band-pass on both signals
	const float tau = math::max(_param_mc_id_tau.get(), 0.f);
	_id_tq_lag += (torque - _id_tq_lag) * (dt / (tau + dt));
	const Vector3f u = _id_u_fast.update(_id_tq_lag, dt) - _id_u_slow.update(_id_tq_lag, dt);
	const Vector3f y = _id_y_fast.update(accel, dt) - _id_y_slow.update(accel, dt);

	// Only the linear range on every axis: beyond MC_ID_TQLIM rotors reach idle/full and the torque is no longer
	// delivered (SITL Mk-7 yaw: rotors at 0 above ~0.3; those samples halved the estimate), plus 0.5 s after while
	// the band-pass still carries the saturated part.
	const float tq_lim = _param_mc_id_tqlim.get();
	bool linear = true;

	for (int i = 0; i < 3; i++) {
		linear = linear && fabsf(torque(i)) < tq_lim && fabsf(_id_tq_lag(i)) < tq_lim;
	}

	if (!linear) {
		_id_sat_hold = 0.5f;

	} else if (_id_sat_hold > 0.f) {
		_id_sat_hold -= dt;
	}

	const float window = math::max(_param_mc_id_t.get(), 1.f);
	const float lambda = expf(-dt / window);

	// hover trim: in a quiet hover (all rates < 2 deg/s) the steady roll/pitch torque is the trim + the integral
	if (rates.norm() < math::radians(2.f)) {
		const float a = dt / 5.f;	// 5 s average
		_id_trim[0] += a * (_param_mc_tq_trim_r.get() + rate_integral(0) - _id_trim[0]);
		_id_trim[1] += a * (_param_mc_tq_trim_p.get() + rate_integral(1) - _id_trim[1]);
		_id_trim_quiet_s += dt;
	}

	for (int i = 0; i < 3; i++) {
		_id_excitation_s[i] *= lambda;

		// excitation measured as the angular acceleration the torque should give (|u| * b): the same threshold
		// fits axes whose effectiveness differs by 10x (Mk-7 SITL: roll/pitch ~4, yaw ~0.6 rad/s^2 per unit)
		if (fabsf(u(i)) * _id_b[i] > _param_mc_id_exc.get() && linear && _id_sat_hold <= 0.f) {
			// scalar RLS with forgetting (only while excited, so the covariance does not wind up in a hover)
			const float k = _id_p[i] * u(i) / (1.f + _id_p[i] * u(i) * u(i));
			const float b_new = _id_b[i] + k * (y(i) - _id_b[i] * u(i));
			_id_p[i] = math::min((1.f - k * u(i)) * _id_p[i] / lambda, P0);

			if (b_new > 0.02f && b_new < 50.f) {	// physically possible effectiveness only
				_id_b[i] = b_new;
				_id_excitation_s[i] += dt;
			}
		}

		_id_excitation_max[i] = math::max(_id_excitation_max[i], _id_excitation_s[i]);
	}

	const hrt_abstime now = hrt_absolute_time();

	if (now - _id_last_pub > 50_ms) {
		effectiveness_id_status_s st{};
		st.active = true;

		for (int i = 0; i < 3; i++) {
			st.converged[i] = _id_excitation_s[i] >= kMinExcitation;
			st.b_est[i] = _id_b[i];
			st.b_var[i] = _id_p[i];
			st.excitation_s[i] = _id_excitation_s[i];
			st.b_confirmed[i] = bc[i];
			st.u_bp[i] = u(i);
			st.y_bp[i] = y(i);
		}

		st.linear = linear && _id_sat_hold <= 0.f;
		st.timestamp = now;
		_effectiveness_id_status_pub.publish(st);
		_id_last_pub = now;
	}
}

void MulticopterRateControl::updateActuatorControlsStatus(const vehicle_torque_setpoint_s &vehicle_torque_setpoint,
		float dt)
{
	for (int i = 0; i < 3; i++) {
		_control_energy[i] += vehicle_torque_setpoint.xyz[i] * vehicle_torque_setpoint.xyz[i] * dt;
	}

	_energy_integration_time += dt;

	if (_energy_integration_time > 500e-3f) {

		actuator_controls_status_s status;
		status.timestamp = vehicle_torque_setpoint.timestamp;

		for (int i = 0; i < 3; i++) {
			status.control_power[i] = _control_energy[i] / _energy_integration_time;
			_control_energy[i] = 0.f;
		}

		_actuator_controls_status_pub.publish(status);
		_energy_integration_time = 0.f;
	}
}

int MulticopterRateControl::task_spawn(int argc, char *argv[])
{
	bool vtol = false;

	if (argc > 1) {
		if (strcmp(argv[1], "vtol") == 0) {
			vtol = true;
		}
	}

	MulticopterRateControl *instance = new MulticopterRateControl(vtol);

	if (instance) {
		desc.object.store(instance);
		desc.task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	desc.object.store(nullptr);
	desc.task_id = -1;

	return PX4_ERROR;
}

int MulticopterRateControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int MulticopterRateControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This implements the multicopter rate controller. It takes rate setpoints (in acro mode
via `manual_control_setpoint` topic) as inputs and outputs actuator control messages.

The controller has a PID loop for angular rate error.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("mc_rate_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_ARG("vtol", "VTOL mode", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int mc_rate_control_main(int argc, char *argv[])
{
	return ModuleBase::main(MulticopterRateControl::desc, argc, argv);
}
