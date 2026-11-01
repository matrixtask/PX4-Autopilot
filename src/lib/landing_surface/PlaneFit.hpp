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
 * @file PlaneFit.hpp
 * Ground plane under the vehicle from 3-D points (terrain camera depth image or a multi-beam range sensor).
 *
 * Points are given in a level, north-east-down frame centred on the vehicle origin. The plane is z = a x + b y + c
 * (z down), fitted by least squares. Outliers (posts, people, a parked car at the edge of the pad) are removed with a
 * robust cut, 3 x 1.4826 x median absolute residual, then the fit is repeated twice. Results:
 *   height          c: vertical distance from the vehicle origin down to the plane
 *   slope           atan(|grad|), slope_azimuth = heading of steepest descent
 *   gear clearance  vertical gap between each landing-gear contact point (body frame, rotated by the attitude) and
 *                   the plane under it; the smallest one is what touches first on a slope.
 * The same header is meant for the companion computer that processes the real camera, so the flight code and the
 * simulation use one definition.
 */

#pragma once

#include <cmath>
#include <cstddef>
#include <algorithm>
#include <matrix/math.hpp>

namespace landing_surface
{

struct Plane {
	bool valid{false};
	float a{0.f};          ///< dz/dx (north)
	float b{0.f};          ///< dz/dy (east)
	float c{NAN};          ///< plane z at the vehicle origin = height [m]
	float slope{NAN};      ///< [rad]
	float slope_azimuth{NAN}; ///< [rad] heading of steepest descent
	float rms{NAN};        ///< [m] residual RMS of the inliers
	int inliers{0};

	float zAt(float x, float y) const { return a * x + b * y + c; }
};

static constexpr float kOutlierSigma = 3.f;
static constexpr float kOutlierFloor = 0.15f; ///< [m] never reject points closer than this to the plane
static constexpr int kMinInliers = 30;

namespace detail
{
inline bool solve(const float *x, const float *y, const float *z, const bool *use, size_t n, Plane &p)
{
	double sxx = 0, sxy = 0, syy = 0, sx = 0, sy = 0, sn = 0, sxz = 0, syz = 0, sz = 0;

	for (size_t i = 0; i < n; i++) {
		if (!use[i]) { continue; }

		const double xi = static_cast<double>(x[i]), yi = static_cast<double>(y[i]), zi = static_cast<double>(z[i]);
		sxx += xi * xi; sxy += xi * yi; syy += yi * yi;
		sx += xi; sy += yi; sn += 1.0;
		sxz += xi * zi; syz += yi * zi; sz += zi;
	}

	if (sn < static_cast<double>(kMinInliers)) { return false; }

	// normal equations [sxx sxy sx; sxy syy sy; sx sy n] [a b c]' = [sxz syz sz]' (Cramer)
	const double m[3][3] = {{sxx, sxy, sx}, {sxy, syy, sy}, {sx, sy, sn}};
	const double r[3] = {sxz, syz, sz};
	auto det3 = [](const double q[3][3]) {
		return q[0][0] * (q[1][1] * q[2][2] - q[1][2] * q[2][1]) - q[0][1] * (q[1][0] * q[2][2] - q[1][2] * q[2][0])
		       + q[0][2] * (q[1][0] * q[2][1] - q[1][1] * q[2][0]);
	};
	const double d = det3(m);

	if (std::fabs(d) < 1e-9 * (sn * sn * sn + 1.0)) { return false; } // points on a line: no plane

	double sol[3];

	for (int k = 0; k < 3; k++) {
		double q[3][3];

		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) { q[i][j] = (j == k) ? r[i] : m[i][j]; }
		}

		sol[k] = det3(q) / d;
	}

	p.a = static_cast<float>(sol[0]);
	p.b = static_cast<float>(sol[1]);
	p.c = static_cast<float>(sol[2]);
	double ss = 0;

	for (size_t i = 0; i < n; i++) {
		if (use[i]) { const double e = static_cast<double>(z[i] - p.zAt(x[i], y[i])); ss += e * e; }
	}

	p.rms = static_cast<float>(std::sqrt(ss / sn));
	p.inliers = static_cast<int>(sn);
	return true;
}
} // namespace detail

/**
 * Fit the ground plane. x, y, z: level NED coordinates of the points relative to the vehicle origin [m];
 * use, scratch: caller-provided work arrays of n entries. Robust (median-based) outlier rejection, three fits.
 */
inline Plane fit(const float *x, const float *y, const float *z, bool *use, float *scratch, size_t n)
{
	Plane p{};

	for (size_t i = 0; i < n; i++) { use[i] = std::isfinite(x[i]) && std::isfinite(y[i]) && std::isfinite(z[i]); }

	for (int pass = 0; pass < 3; pass++) {
		if (!detail::solve(x, y, z, use, n, p)) { return Plane{}; }

		if (pass == 2) { break; }

		size_t m = 0;

		for (size_t i = 0; i < n; i++) {
			if (use[i]) { scratch[m++] = std::fabs(z[i] - p.zAt(x[i], y[i])); }
		}

		std::nth_element(scratch, scratch + m / 2, scratch + m);
		const float mad = scratch[m / 2];
		const float limit = fmaxf(kOutlierSigma * 1.4826f * mad, kOutlierFloor);

		for (size_t i = 0; i < n; i++) {
			if (use[i] && std::fabs(z[i] - p.zAt(x[i], y[i])) > limit) { use[i] = false; }
		}
	}

	p.valid = true;
	p.slope = atanf(sqrtf(p.a * p.a + p.b * p.b));
	p.slope_azimuth = atan2f(p.b, p.a); // z (down) grows fastest along (a, b): that is downhill
	return p;
}

/**
 * Smallest vertical gap between the landing-gear contact points and the plane.
 * gear_body: contact points in the body frame (FRD) relative to the vehicle origin; q: attitude (body -> NED).
 */
inline float gearClearance(const Plane &p, const matrix::Quatf &q, const matrix::Vector3f *gear_body, size_t n_gear)
{
	if (!p.valid) { return NAN; }

	float clearance = INFINITY;

	for (size_t i = 0; i < n_gear; i++) {
		const matrix::Vector3f g = q.rotateVector(gear_body[i]);
		clearance = fminf(clearance, p.zAt(g(0), g(1)) - g(2));
	}

	return clearance;
}

} // namespace landing_surface
