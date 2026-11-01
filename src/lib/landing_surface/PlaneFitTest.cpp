/****************************************************************************
 *
 *   Copyright (c) 2026 Tetra Aviation Corp. All rights reserved.
 *   BSD-3-Clause, same terms as PX4.
 *
 ****************************************************************************/

/**
 * @file PlaneFitTest.cpp
 * Ground-plane fit for landing: slope, height, outlier rejection, gear clearance on a slope.
 */

#include <gtest/gtest.h>
#include <lib/landing_surface/PlaneFit.hpp>

using namespace landing_surface;

namespace
{
// a 5 deg slope descending towards the east, 20 m below the vehicle, sampled on a 20 x 20 grid
void makeSlope(float slope_deg, float height, float *x, float *y, float *z, size_t &n)
{
	n = 0;
	const float b = tanf(slope_deg * M_PI_F / 180.f);

	for (int i = 0; i < 20; i++) {
		for (int j = 0; j < 20; j++) {
			x[n] = -5.f + 0.5f * i;
			y[n] = -5.f + 0.5f * j;
			z[n] = height + b * y[n] + 0.01f * ((i * 7 + j * 3) % 5 - 2); // +-2 cm texture
			n++;
		}
	}
}
} // namespace

TEST(PlaneFit, SlopeHeightAndDirection)
{
	float x[400], y[400], z[400];
	bool use[400];
	float scratch[400];
	size_t n;
	makeSlope(5.f, 20.f, x, y, z, n);
	const Plane p = fit(x, y, z, use, scratch, n);
	ASSERT_TRUE(p.valid);
	EXPECT_NEAR(p.c, 20.f, 0.02f);
	EXPECT_NEAR(p.slope * 180.f / M_PI_F, 5.f, 0.1f);
	EXPECT_NEAR(p.slope_azimuth, M_PI_F / 2.f, 0.02f); // downhill = east
}

TEST(PlaneFit, RejectsAnObstacle)
{
	float x[400], y[400], z[400];
	bool use[400];
	float scratch[400];
	size_t n;
	makeSlope(0.f, 10.f, x, y, z, n);

	for (size_t i = 0; i < 40; i++) { z[i] -= 1.5f; } // a 1.5 m box on part of the pad

	const Plane p = fit(x, y, z, use, scratch, n);
	ASSERT_TRUE(p.valid);
	EXPECT_NEAR(p.c, 10.f, 0.05f);
	EXPECT_LT(p.slope * 180.f / M_PI_F, 0.5f);
	EXPECT_LE(p.inliers, 360);
}

TEST(PlaneFit, GearClearanceOnASlope)
{
	float x[400], y[400], z[400];
	bool use[400];
	float scratch[400];
	size_t n;
	makeSlope(5.f, 2.f, x, y, z, n); // ground 2 m below the origin at the centre, falling towards +y (east)
	const Plane p = fit(x, y, z, use, scratch, n);
	// level vehicle, main wheels at y = +-0.91 m, 0.8 m below the origin
	const matrix::Vector3f gear[2] = {{0.f, -0.91f, 0.8f}, {0.f, 0.91f, 0.8f}};
	const float c = gearClearance(p, matrix::Quatf{}, gear, 2);
	// uphill (west) wheel: 2 - 0.8 - 0.91 tan 5 deg = 1.120 m
	EXPECT_NEAR(c, 2.f - 0.8f - 0.91f * tanf(5.f * M_PI_F / 180.f), 0.02f);
}

TEST(PlaneFit, TooFewPoints)
{
	float x[10] = {}, y[10] = {}, z[10] = {}, scratch[10];
	bool use[10];
	EXPECT_FALSE(fit(x, y, z, use, scratch, 10).valid);
}
