/****************************************************************************
 *
 *   Copyright (c) 2026 Tetra Aviation Corp. All rights reserved.
 *   BSD-3-Clause, same terms as PX4.
 *
 ****************************************************************************/

/**
 * @file BackTransitionBrakeTest.cpp
 * Checks the C++ law against its Python mirror (mk7_mission/brake_ff.py, the one the back-transition simulation
 * tuned) and the physical properties the back transition relies on.
 */

#include <gtest/gtest.h>
#include <lib/vtol_brake/BackTransitionBrake.hpp>

using namespace vtol_brake;

namespace
{
struct Reference {
	float speed, vz, decel, pitch_cmd;
	float ideal_pitch, spoiler, lift, sink_max;
};

// generated with mk7_mission/brake_ff.py, default BrakeParams (EM3)
const Reference kReference[] = {
	{72.f, 0.f, 2.f, 0.09f, 0.05545f, 0.00000f, 16189.4f, 11.1433f},
	{50.f, 3.f, 2.f, 0.12f, 0.07978f, 0.45647f, 11033.6f, 4.9031f},
	{30.f, 1.f, 2.5f, 0.18f, 0.19391f, 1.00000f, 3042.5f, 0.6441f},
	{12.f, 3.f, 2.f, 0.2f, 0.20297f, 1.00000f, 686.0f, 28.1591f},
};
} // namespace

TEST(BackTransitionBrake, MatchesPythonMirror)
{
	const Params p{};

	for (const Reference &r : kReference) {
		EXPECT_NEAR(idealPitch(p, r.speed, r.vz, r.decel), r.ideal_pitch, 2e-3f) << r.speed;
		const Output out = brake(p, r.speed, r.vz, r.decel, r.pitch_cmd);
		EXPECT_NEAR(out.spoiler, r.spoiler, 2e-3f) << r.speed;
		EXPECT_NEAR(out.lift, r.lift, 0.002f * p.mass * kGravity) << r.speed;
		EXPECT_NEAR(sinkMax(p, r.speed, r.decel), r.sink_max, 0.02f * r.sink_max + 0.02f) << r.speed;
	}
}

TEST(BackTransitionBrake, WingNeverCarriesMoreThanItsShare)
{
	// at the ideal pitch the rotors keep at least r_min of the weight: no balloon by construction
	const Params p{};
	const float weight = p.mass * kGravity;

	for (float v = 20.f; v <= 80.f; v += 5.f) {
		const float pitch = idealPitch(p, v, 0.f, 2.f);
		const Output out = brake(p, v, 0.f, 2.f, pitch);
		EXPECT_GE(out.fz, (p.r_min - 0.02f) * weight) << v;
	}
}

TEST(BackTransitionBrake, SpoilerRisesWithDemand)
{
	const Params p{};
	float last = -1.f;

	for (float decel = 0.5f; decel <= 3.f; decel += 0.5f) {
		const Output out = brake(p, 60.f, 0.f, decel, 0.09f);
		EXPECT_GE(out.spoiler, last);
		last = out.spoiler;
	}
}

TEST(BackTransitionBrake, MeetsDemandWhenFeasible)
{
	const Params p{};

	for (float v = 15.f; v <= 72.f; v += 3.f) {
		const float pitch = idealPitch(p, v, 0.f, 2.f);
		const Output out = brake(p, v, 0.f, 2.f, pitch);

		// spoiler not saturated at either end (near 72 m/s the airframe drag alone exceeds the demand)
		if (out.spoiler > 0.001f && out.spoiler < 0.999f) {
			EXPECT_NEAR(out.decel, 2.f, 0.02f) << v;
		}
	}
}
