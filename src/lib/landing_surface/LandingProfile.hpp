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
 * @file LandingProfile.hpp
 * Vertical landing on a measured surface: the descent-speed envelope and the landing-site search pattern.
 *
 * Descent (gear height h = clearance of the lowest landing-gear contact point above the ground):
 *
 *   v_limit (VRS chart) --brake--> v_approach at h_approach --brake--> v_crawl at h_crawl --> flare: v_crawl ->
 *   v_flare linearly over the last h_flare
 *
 * At contact (gear height <= kContactHeight) the crawl speed is commanded again: the multicopter land detector only
 * sees ground contact while a descent of at least 1.1 LNDMC_Z_VEL_MAX is commanded, so the flare speed alone would
 * leave the vehicle standing on the ground without a landing (SITL: 10 min). The impact speed is set by the flare.
 *
 * The large reduction (v_limit -> v_approach) is a jerk-limited braking curve (math::trajectory::computeMaxSpeedFromDistance)
 * ending at the gate, so the smoothed setpoint has slowed down by the time the gate is reached instead of crossing it at
 * speed. The small step v_approach -> v_crawl uses constant deceleration: the jerk-limited formula is so conservative
 * at low jerk (MPC_JERK_AUTO 0.5) that the approach segment would vanish (0.3 m/s from 2 m above the crawl gate).
 *
 * Site search: candidate points on rings of radius step, 2 step, ... up to max_radius around the original pad.
 * Each ring starts in the downhill direction of the rejected surface and fans out to both sides (0, +1, -1, +2, ...),
 * since the ground usually flattens downhill of a slope and the nearest acceptable point is the cheapest to reach.
 */

#pragma once

#include <cmath>
#include <lib/mathlib/mathlib.h>
#include <lib/mathlib/math/TrajMath.hpp>
#include <matrix/math.hpp>

namespace landing_surface
{

static constexpr float kContactHeight = 0.05f; ///< [m] gear height taken as ground contact (measurement noise)

struct DescentProfile {
	float v_limit;     ///< [m/s] descent limit away from the ground (vortex-ring-state chart)
	float v_approach;  ///< [m/s] approach speed
	float h_approach;  ///< [m] gear height at which v_approach is reached
	float v_crawl;     ///< [m/s] final descent speed
	float h_crawl;     ///< [m] gear height at which v_crawl is reached
	float h_flare;     ///< [m] gear height at which the flare starts
	float v_flare;     ///< [m/s] sink rate at contact
	float accel;       ///< [m/s^2] braking deceleration
	float jerk;        ///< [m/s^3] braking jerk
};

/** Maximum descent speed [m/s, positive down] at gear height h [m]. */
inline float descentSpeed(const DescentProfile &p, float h)
{
	if (h <= kContactHeight) {
		return p.v_crawl; // settle: let the land detector see a commanded descent without movement
	}

	const float to_approach = h - p.h_approach;
	const float to_crawl = h - p.h_crawl;

	float v = p.v_limit;
	v = math::min(v, to_approach > 0.f
		      ? math::trajectory::computeMaxSpeedFromDistance(p.jerk, p.accel, to_approach, p.v_approach)
		      : p.v_approach);
	v = math::min(v, to_crawl > 0.f ? sqrtf(p.v_crawl * p.v_crawl + 2.f * p.accel * to_crawl) : p.v_crawl);

	if (h < p.h_flare && p.h_flare > FLT_EPSILON) {
		const float s = math::max(h, 0.f) / p.h_flare;
		v = math::min(v, p.v_flare + (p.v_crawl - p.v_flare) * s);
	}

	return math::max(v, p.v_flare);
}

/**
 * Offset (north, east) [m] of search candidate `index` (0, 1, ...) from the original pad.
 * Ring k (1, 2, ...) has radius k step and max(6, round(2 pi k)) points (spacing about one step).
 * @return false when index is beyond max_radius (search exhausted)
 */
inline bool searchCandidate(int index, float step, float max_radius, float downhill_azimuth, matrix::Vector2f &offset)
{
	if (index < 0 || step < 1.f) {
		return false;
	}

	for (int ring = 1; static_cast<float>(ring) * step <= max_radius + 0.01f; ring++) {
		const int count = math::max(6, static_cast<int>(roundf(2.f * M_PI_F * static_cast<float>(ring))));

		if (index < count) {
			// 0, +1, -1, +2, -2, ... sectors from downhill
			const int k = (index + 1) / 2;
			const float sign = (index % 2 == 1) ? 1.f : -1.f;
			const float azimuth = downhill_azimuth + sign * static_cast<float>(k) * 2.f * M_PI_F / static_cast<float>(count);
			const float r = static_cast<float>(ring) * step;
			offset = matrix::Vector2f(r * cosf(azimuth), r * sinf(azimuth));
			return true;
		}

		index -= count;
	}

	return false;
}

} // namespace landing_surface
