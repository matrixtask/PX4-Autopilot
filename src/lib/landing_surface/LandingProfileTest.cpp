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

#include <gtest/gtest.h>

#include "LandingProfile.hpp"

#include <px4_platform_common/defines.h>

using namespace landing_surface;

namespace
{
// Mk-7 airframe 22002: VRS limit 3 m/s, approach 0.5 m/s from 3 m, crawl 0.3 m/s from 1 m, flare 0.3 m to 0.1 m/s
constexpr DescentProfile kMk7{3.f, 0.5f, 3.f, 0.3f, 1.f, 0.3f, 0.1f, 1.f, 0.5f};
}

TEST(LandingProfile, GatesAndFlare)
{
	EXPECT_NEAR(descentSpeed(kMk7, 100.f), 3.f, 1e-4f);    // far from the ground: the VRS limit
	EXPECT_NEAR(descentSpeed(kMk7, 3.f), 0.5f, 1e-4f);     // approach gate
	EXPECT_NEAR(descentSpeed(kMk7, 2.f), 0.5f, 0.05f);     // approach segment (braking towards the crawl gate)
	EXPECT_NEAR(descentSpeed(kMk7, 1.f), 0.3f, 1e-4f);     // crawl gate
	EXPECT_NEAR(descentSpeed(kMk7, 0.15f), 0.2f, 1e-4f);   // half way through the flare
	EXPECT_NEAR(descentSpeed(kMk7, 0.06f), 0.14f, 1e-4f);  // just before contact
	EXPECT_NEAR(descentSpeed(kMk7, 0.f), 0.3f, 1e-4f);     // contact: crawl again so the land detector can confirm
	EXPECT_NEAR(descentSpeed(kMk7, -0.2f), 0.3f, 1e-4f);   // below the plane (noise)
}

TEST(LandingProfile, MonotonicInHeight)
{
	float previous = 0.f;

	for (float h = kContactHeight + 0.001f; h < 60.f; h += 0.01f) { // above contact
		const float v = descentSpeed(kMk7, h);
		EXPECT_GE(v, previous - 1e-5f) << "h " << h;
		previous = v;
	}
}

TEST(LandingProfile, JerkLimitedFollowerTouchesDownAtTheFlareSpeed)
{
	// a vertical setpoint follower with the airframe's limits (accel 1.5 m/s^2, jerk 0.5 m/s^3), starting at the VRS
	// limit 40 m above the pad; the old profile (interpolation 10 m -> 5 m, then 0.3 below 1 m) reached the ground at
	// 0.8-0.9 m/s with these limits (HQ01 SITL)
	float h = 40.f, v = 3.f, a = 0.f;
	const float dt = 0.01f, a_max = 1.5f, jerk = 0.5f, gain = 2.f;
	float t = 0.f;
	float v_at_crawl_gate = NAN;

	while (h > 0.f && t < 200.f) {
		if (!PX4_ISFINITE(v_at_crawl_gate) && h <= kMk7.h_flare) {
			v_at_crawl_gate = v; // speed when the flare starts: the 0.3 m/s step must be done by then
		}

		const float v_sp = descentSpeed(kMk7, h);
		const float a_sp = math::constrain(gain * (v_sp - v), -a_max, a_max);
		a += math::constrain(a_sp - a, -jerk * dt, jerk * dt);
		v += a * dt;
		h -= v * dt;
		t += dt;
	}

	EXPECT_LE(v_at_crawl_gate, 0.33f) << "at the flare " << v_at_crawl_gate;
	EXPECT_LE(v, 0.2f) << "touchdown " << v;
	EXPECT_LT(t, 40.f) << "time " << t;
}

TEST(LandingProfile, SearchRingsStartDownhill)
{
	matrix::Vector2f o;
	const float east = M_PI_F / 2.f;

	ASSERT_TRUE(searchCandidate(0, 12.f, 40.f, east, o));
	EXPECT_NEAR(o(0), 0.f, 1e-3f);   // first candidate: 12 m downhill (east)
	EXPECT_NEAR(o(1), 12.f, 1e-3f);

	ASSERT_TRUE(searchCandidate(1, 12.f, 40.f, east, o));
	EXPECT_NEAR(o.norm(), 12.f, 1e-3f);  // then 60 deg to one side
	EXPECT_NEAR(atan2f(o(1), o(0)), east + M_PI_F / 3.f, 1e-3f);

	ASSERT_TRUE(searchCandidate(6, 12.f, 40.f, east, o));   // ring 1 has 6 points; index 6 starts ring 2
	EXPECT_NEAR(o.norm(), 24.f, 1e-3f);
	EXPECT_NEAR(o(1), 24.f, 1e-3f);

	// rings 1..3 (12, 24, 36 m): 6 + 13 + 19 = 38 candidates, then exhausted
	ASSERT_TRUE(searchCandidate(37, 12.f, 40.f, east, o));
	EXPECT_NEAR(o.norm(), 36.f, 1e-3f);
	EXPECT_FALSE(searchCandidate(38, 12.f, 40.f, east, o));
	EXPECT_FALSE(searchCandidate(0, 0.f, 40.f, east, o));
}
