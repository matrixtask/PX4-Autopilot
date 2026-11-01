/****************************************************************************
 *
 *   Copyright (c) 2026 Tetra Aviation Corp. All rights reserved.
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
 * @file BackTransitionBrake.hpp
 * Feed-forward brake law for the back transition of a standard VTOL with spoilers.
 *
 * Longitudinal force balance in the horizontal/vertical frame, flight-path angle g (climb > 0), pitch t:
 *   rotors  thrust T along body-up: horizontal -T sin t, vertical Fz = T cos t
 *   wing    lift L normal to the path, drag D along -path (spoiler drag and a windmilling pusher included)
 *   decel   m a = D cos g + L sin g + Fz tan t,   Fz = W - L cos g + D sin g
 *
 * The rotors take a speed-scheduled share r of the weight (r = 1 - (V/V_lift)^2, at least r_min). The pitch is
 * set so that the wing carries the rest, so the wing cannot balloon: nothing waits for a climb to appear. The
 * spoiler then brings the total deceleration to the demand. If the spoiler is saturated, the pitch rises further
 * (rotor thrust tilted back brakes) up to the angle-of-attack / tilt limit.
 * Callers command the pitch as horizontal acceleration g tan(t) and feed the wing lift forward as vertical
 * acceleration +L/m (NED), so the rotors produce W - L directly.
 *
 * The same law, line for line, is mk7_mission/brake_ff.py; tools/bt_sim.py tunes it in a back-transition-only
 * simulation before it is flown (Mk-7 EM3: 72 m/s to a stop in 34 s / 1.3 km at 0.2 g, see docs/bt_sim).
 */

#pragma once

#include <cmath>
#include <lib/mathlib/mathlib.h>
#include <px4_platform_common/defines.h>

namespace vtol_brake
{

struct Params {
	float mass{1560.f};                 ///< [kg]
	float rho{1.225f};                  ///< [kg/m^3]
	float scla{56.93f};                 ///< wing area * lift slope [m^2/rad]
	float scl0{-0.025f};                ///< wing area * CL0 [m^2]
	float alpha_stall{0.339f};          ///< [rad]
	float scla_stall{-46.2f};           ///< wing area * post-stall lift slope [m^2/rad]
	float spcl{6.60f};                  ///< spoiler lift loss at full deflection [m^2]
	float spcd{1.27f};                  ///< spoiler drag area at full deflection [m^2]
	float sp_angle{0.785f};             ///< full spoiler deflection [rad] (drag ~ sin(s * sp_angle))
	float cda{0.91f};                   ///< parasite + windmilling-propeller drag area [m^2]
	float kind{170.9f};                 ///< induced-drag area S pi AR e [m^2]: Di = L^2 / (q kind)
	float v_lift{72.f};                 ///< speed above which the wing carries the whole weight [m/s]
	float r_min{0.35f};                 ///< least rotor share of the weight (keeps lift rotors loaded)
	float alpha_max{0.244f};            ///< wing angle-of-attack cap while wing-borne [rad]
	float pitch_max{0.314f};            ///< [rad] (MPC_TILTMAX_AIR)
	float pitch_min{-0.052f};           ///< [rad]
	float v_alpha_lo{10.f};             ///< alpha cap fades out between v_alpha_hi and v_alpha_lo: below ~15 m/s
	float v_alpha_hi{25.f};             ///< the wing carries < 15 % of the weight, a high alpha is harmless
	float gamma_margin{0.035f};         ///< alpha kept in reserve when limiting the sink [rad]
	float lift_scale{1.f};              ///< online correction of the wing lift (observeLiftScale)
};

struct Output {
	float pitch{0.f};    ///< [rad] nose up > 0
	float spoiler{0.f};  ///< [0, 1]
	float lift{0.f};     ///< [N] predicted wing lift (spoiler included)
	float fz{0.f};       ///< [N] vertical force the rotors must produce
	float decel{0.f};    ///< [m/s^2] predicted deceleration
};

static constexpr float kGravity = 9.80665f;

inline float alphaCap(const Params &p, float speed)
{
	const float k = math::constrain((speed - p.v_alpha_lo) / (p.v_alpha_hi - p.v_alpha_lo), 0.f, 1.f);
	return p.alpha_max + (1.f - k) * (math::radians(90.f) - p.alpha_max);
}

struct Forces {
	float lift;
	float drag;
	float fz;
	float decel;
};

inline Forces forces(const Params &p, float q, float gamma, float pitch, float spoiler)
{
	const float alpha = pitch - gamma;
	const float a_lin = math::constrain(alpha, -p.alpha_stall, p.alpha_stall);
	const float lift_clean = q * p.lift_scale * (p.scl0 + p.scla * a_lin + p.scla_stall * (alpha - a_lin));
	const float lift = lift_clean - q * p.spcl * spoiler;
	const float drag = q * (p.cda + p.spcd * sinf(spoiler * p.sp_angle) / sinf(p.sp_angle))
			   + lift_clean * lift_clean / (q * p.kind);
	const float weight = p.mass * kGravity;
	const float fz = weight - lift * cosf(gamma) + drag * sinf(gamma);
	const float decel = (drag * cosf(gamma) + lift * sinf(gamma) + fz * tanf(pitch)) / p.mass;
	return Forces{lift, drag, fz, decel};
}

/** Spoiler for a given pitch. Deceleration rises monotonically with the spoiler (more drag, less lift). */
inline float solveSpoiler(const Params &p, float q, float gamma, float pitch, float decel)
{
	if (forces(p, q, gamma, pitch, 0.f).decel >= decel) { return 0.f; }

	if (forces(p, q, gamma, pitch, 1.f).decel <= decel) { return 1.f; }

	float lo = 0.f;
	float hi = 1.f;

	for (int i = 0; i < 20; i++) {
		const float mid = 0.5f * (lo + hi);

		if (forces(p, q, gamma, pitch, mid).decel < decel) {
			lo = mid;

		} else {
			hi = mid;
		}
	}

	return 0.5f * (lo + hi);
}

inline float flightPathAngle(float speed, float vz_ned) { return atan2f(-vz_ned, speed); }

inline float dynamicPressure(const Params &p, float speed) { return 0.5f * p.rho * speed * speed; }

/** Pitch the law aims for before rate limiting. speed: airspeed [m/s], vz_ned [m/s], decel demand [m/s^2]. */
inline float idealPitch(const Params &p, float speed, float vz_ned, float decel)
{
	const float v = math::max(speed, 1.f);
	const float q = dynamicPressure(p, v);
	const float gamma = flightPathAngle(v, vz_ned);
	const float weight = p.mass * kGravity;
	const float share = math::constrain(1.f - (v / p.v_lift) * (v / p.v_lift), p.r_min, 1.f);
	const float lift_target = (1.f - share) * weight / math::max(cosf(gamma), 0.5f);
	const float alpha_cap = alphaCap(p, v);

	auto pitch_for = [&](float spoiler) {
		const float alpha = (lift_target / (q * p.lift_scale) - p.scl0 + p.spcl * spoiler / p.lift_scale) / p.scla;
		return math::constrain(gamma + math::min(alpha, alpha_cap), p.pitch_min, p.pitch_max);
	};

	if (forces(p, q, gamma, pitch_for(1.f), 1.f).decel < decel) {
		// spoiler saturated: pitch further up so the tilted rotor thrust brakes, up to the alpha / tilt cap
		float pitch = pitch_for(1.f);
		const float top = math::min(gamma + alpha_cap, p.pitch_max);
		const float step = math::radians(0.25f);

		while (pitch + step <= top && forces(p, q, gamma, pitch, 1.f).decel < decel) {
			pitch += step;
		}

		return pitch;
	}

	if (forces(p, q, gamma, pitch_for(0.f), 0.f).decel >= decel) {
		return pitch_for(0.f);
	}

	float lo = 0.f;
	float hi = 1.f;

	for (int i = 0; i < 20; i++) {
		const float mid = 0.5f * (lo + hi);

		if (forces(p, q, gamma, pitch_for(mid), mid).decel < decel) {
			lo = mid;

		} else {
			hi = mid;
		}
	}

	return pitch_for(0.5f * (lo + hi));
}

/**
 * Steepest sink rate [m/s, positive down] that still leaves the angle of attack needed to brake: braking first,
 * descent with what is left. Descending steepens the path; with the alpha capped the nose would have to come
 * down and the rotor brake would disappear.
 */
inline float sinkMax(const Params &p, float speed, float decel)
{
	const float v = math::max(speed, 1.f);
	const float pitch_level = idealPitch(p, v, 0.f, decel);
	const float room = alphaCap(p, v) - math::max(pitch_level, 0.f) - p.gamma_margin;
	return v * tanf(math::constrain(room, 0.f, math::radians(80.f)));
}

/**
 * Lift observer: corrects the wing-lift model online (model error, not a climb detector). Vertical balance
 *   m (g + a_up) = Fz_rotor + L cos(g) - D sin(g),   L = k L_clean(model) - q spcl s
 * solved for k, low-passed (tau) and limited to 0.6 .. 2.5. Only while the wing carries a meaningful share
 * (q > 600 Pa and model lift > 0.1 W); otherwise k is kept. Mk-7 EM3 SITL HQ01 run 3: the wing gave ~1.7x the
 * model lift with the spoiler retracted and the aircraft climbed 22 m with the rotors at idle.
 */
inline float observeLiftScale(const Params &p, float speed, float vz_ned, float pitch, float spoiler, float accel_up,
			      float rotor_fz, float k_prev, float dt, float tau = 1.f)
{
	const float v = math::max(speed, 1.f);
	const float q = dynamicPressure(p, v);

	if (q < 600.f || !PX4_ISFINITE(accel_up) || !PX4_ISFINITE(rotor_fz) || !PX4_ISFINITE(pitch)) {
		return k_prev;
	}

	const float gamma = flightPathAngle(v, vz_ned);
	const float alpha = pitch - gamma;
	const float a_lin = math::constrain(alpha, -p.alpha_stall, p.alpha_stall);
	const float lift_clean = q * (p.scl0 + p.scla * a_lin + p.scla_stall * (alpha - a_lin));

	if (lift_clean < 0.1f * p.mass * kGravity) {
		return k_prev;
	}

	Params unit = p;
	unit.lift_scale = 1.f;
	const Forces f = forces(unit, q, gamma, pitch, spoiler);
	const float lift = (p.mass * (kGravity + accel_up) - rotor_fz + f.drag * sinf(gamma)) / math::max(cosf(gamma), 0.5f);
	const float k = math::constrain((lift + q * p.spcl * spoiler) / lift_clean, 0.6f, 2.5f);
	return k_prev + (k - k_prev) * math::min(dt / tau, 1.f);
}

/** Spoiler, lift and rotor force for the pitch actually commanded (after rate limiting). */
inline Output brake(const Params &p, float speed, float vz_ned, float decel, float pitch_cmd)
{
	const float v = math::max(speed, 1.f);
	const float q = dynamicPressure(p, v);
	const float gamma = flightPathAngle(v, vz_ned);
	const float spoiler = solveSpoiler(p, q, gamma, pitch_cmd, decel);
	const Forces f = forces(p, q, gamma, pitch_cmd, spoiler);
	return Output{pitch_cmd, spoiler, f.lift, f.fz, f.decel};
}

} // namespace vtol_brake
