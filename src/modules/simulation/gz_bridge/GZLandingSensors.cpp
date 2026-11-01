/****************************************************************************
 *
 *   Copyright (c) 2026 Tetra Aviation Corp. All rights reserved.
 *   BSD-3-Clause, same terms as PX4.
 *
 ****************************************************************************/

#include "GZLandingSensors.hpp"

#include <drivers/drv_hrt.h>
#include <drivers/drv_sensor.h>
#include <lib/drivers/device/Device.hpp>
#include <px4_platform_common/log.h>

#include <cmath>
#include <cstring>

bool GZLandingSensors::init(const std::string &world_name, const std::string &model_name)
{
	const std::string base = "/world/" + world_name + "/model/" + model_name + "/link/base_link/sensor/";

	// Optional sensors: a model without them just never publishes on these topics.
	if (!_node.Subscribe(base + "radar_alt/scan", &GZLandingSensors::radarCallback, this)) {
		PX4_WARN("failed to subscribe to %s", (base + "radar_alt/scan").c_str());
	}

	if (!_node.Subscribe(base + "terrain_cam/camera_info", &GZLandingSensors::cameraInfoCallback, this)) {
		PX4_WARN("failed to subscribe to %s", (base + "terrain_cam/camera_info").c_str());
	}

	if (!_node.Subscribe(base + "terrain_cam/depth_image", &GZLandingSensors::depthCallback, this)) {
		PX4_WARN("failed to subscribe to %s", (base + "terrain_cam/depth_image").c_str());
	}

	return true;
}

void GZLandingSensors::radarCallback(const gz::msgs::LaserScan &msg)
{
	// Radar altimeter: the nearest return within the beam.
	float nearest = INFINITY;
	const float range_min = static_cast<float>(msg.range_min());
	const float range_max = static_cast<float>(msg.range_max());

	for (int i = 0; i < msg.ranges_size(); i++) {
		const float r = static_cast<float>(msg.ranges(i));

		if (std::isfinite(r) && r >= range_min && r <= range_max) {
			nearest = fminf(nearest, r);
		}
	}

	device::Device::DeviceId id{};
	id.devid_s.bus_type = device::Device::DeviceBusType::DeviceBusType_SIMULATION;
	id.devid_s.devtype = DRV_DIST_DEVTYPE_SIM;
	id.devid_s.bus = 2;
	id.devid_s.address = 2;

	distance_sensor_s report{};
	report.timestamp = hrt_absolute_time();
	report.device_id = id.devid;
	report.min_distance = range_min;
	report.max_distance = range_max;
	report.current_distance = std::isfinite(nearest) ? nearest : report.max_distance + 1.f; // out of range
	report.variance = 0.05f * 0.05f;
	report.signal_quality = std::isfinite(nearest) ? 100 : 0;
	report.type = distance_sensor_s::MAV_DISTANCE_SENSOR_RADAR;
	report.orientation = distance_sensor_s::ROTATION_DOWNWARD_FACING; // fixed to the airframe
	report.h_fov = 0.21f; // ~12 deg beam
	report.v_fov = 0.21f;
	_radar_pub.publish(report);
}

void GZLandingSensors::cameraInfoCallback(const gz::msgs::CameraInfo &msg)
{
	if (_have_intrinsics.load() || msg.intrinsics().k_size() < 9) {
		return;
	}

	_fx = static_cast<float>(msg.intrinsics().k(0));
	_cx = static_cast<float>(msg.intrinsics().k(2));
	_fy = static_cast<float>(msg.intrinsics().k(4));
	_cy = static_cast<float>(msg.intrinsics().k(5));
	_have_intrinsics.store(_fx > 1.f && _fy > 1.f);
}

void GZLandingSensors::depthCallback(const gz::msgs::Image &msg)
{
	vehicle_attitude_s att{};

	if (!_have_intrinsics.load() || msg.pixel_format_type() != gz::msgs::PixelFormatType::R_FLOAT32
	    || !_attitude_sub.copy(&att)) {
		return;
	}

	const int w = static_cast<int>(msg.width());
	const int h = static_cast<int>(msg.height());

	if (w <= 0 || h <= 0 || msg.data().size() < static_cast<size_t>(w * h) * sizeof(float)) {
		return;
	}

	const size_t n_max = static_cast<size_t>((w / kStride + 1) * (h / kStride + 1));

	if (_capacity < n_max) {
		_x.resize(n_max); _y.resize(n_max); _z.resize(n_max); _scratch.resize(n_max);
		_use.reset(new bool[n_max]);
		_capacity = n_max;
	}

	const matrix::Quatf q{att.q};
	const matrix::Vector3f mount{kCamX, kCamY, kCamZ};
	const float *depth = reinterpret_cast<const float *>(msg.data().data());
	size_t n = 0;

	for (int v = 0; v < h; v += kStride) {
		for (int u = 0; u < w; u += kStride) {
			const float d = depth[v * w + u];

			if (!std::isfinite(d) || d <= 0.f) {
				continue;
			}

			// gz camera frame (X along the view, Y left, Z up); camera looks straight down, image top = forward:
			// view -> body down, image right (-Y_cam) -> body right, image up (+Z_cam) -> body forward (FRD)
			const float right = (static_cast<float>(u) - _cx) / _fx * d;
			const float up = -(static_cast<float>(v) - _cy) / _fy * d;
			const matrix::Vector3f p_body = mount + matrix::Vector3f(up, right, d);
			const matrix::Vector3f p = q.rotateVector(p_body); // level NED, origin at the vehicle
			_x[n] = p(0);
			_y[n] = p(1);
			_z[n] = p(2);
			n++;
		}
	}

	const landing_surface::Plane plane = landing_surface::fit(_x.data(), _y.data(), _z.data(),
					     _use.get(), _scratch.data(), n);

	landing_surface_s out{};
	out.timestamp_sample = hrt_absolute_time();
	out.valid = plane.valid;
	out.height = plane.c;
	out.dzdx = plane.a;
	out.dzdy = plane.b;
	out.slope = plane.slope;
	out.slope_azimuth = plane.slope_azimuth;
	out.roughness = plane.rms;
	out.inliers = static_cast<uint16_t>(plane.inliers);
	out.points = static_cast<uint16_t>(n);
	out.timestamp = hrt_absolute_time();
	_surface_pub.publish(out);
}
