/****************************************************************************
 *
 *   Copyright (c) 2026 Tetra Aviation Corp. All rights reserved.
 *   BSD-3-Clause, same terms as PX4.
 *
 ****************************************************************************/

/**
 * @file GZLandingSensors.hpp
 * Landing sensors of the simulated vehicle, if its model has them (Mk-7 EM3):
 *  - radar altimeter: a downward gpu_lidar named "radar_alt" with a small ray grid standing in for the radar beam
 *    (about 12 deg). The nearest return in the beam is the reading, as for a radar altimeter.
 *    -> distance_sensor (type RADAR, downward)
 *  - terrain camera: a downward depth_camera named "terrain_cam". Pixels are back-projected with the camera
 *    intrinsics (camera_info) and the vehicle attitude, and a ground plane is fitted (lib/landing_surface).
 *    -> landing_surface. On the real aircraft the same fit runs on the companion computer.
 * Models without these sensors are unaffected (the subscriptions simply receive nothing).
 */

#pragma once

#include <gz/msgs.hh>
#include <gz/transport.hh>
#include <gz/msgs/laserscan.pb.h>
#include <gz/msgs/image.pb.h>
#include <gz/msgs/camera_info.pb.h>

#include <lib/landing_surface/PlaneFit.hpp>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/landing_surface.h>
#include <uORB/topics/vehicle_attitude.h>

#include <atomic>
#include <memory>
#include <string>
#include <vector>

class GZLandingSensors
{
public:
	explicit GZLandingSensors(gz::transport::Node &node) : _node(node) {}

	bool init(const std::string &world_name, const std::string &model_name);

private:
	void radarCallback(const gz::msgs::LaserScan &msg);
	void depthCallback(const gz::msgs::Image &msg);
	void cameraInfoCallback(const gz::msgs::CameraInfo &msg);

	gz::transport::Node &_node;

	uORB::PublicationMulti<distance_sensor_s> _radar_pub{ORB_ID(distance_sensor)};
	uORB::Publication<landing_surface_s> _surface_pub{ORB_ID(landing_surface)};
	uORB::Subscription _attitude_sub{ORB_ID(vehicle_attitude)};

	// camera intrinsics (from camera_info)
	std::atomic<bool> _have_intrinsics{false};
	float _fx{0.f}, _fy{0.f}, _cx{0.f}, _cy{0.f};

	// work arrays, sized on the first image
	std::vector<float> _x, _y, _z, _scratch;
	std::unique_ptr<bool[]> _use;
	size_t _capacity{0};

	// terrain camera mount in the body frame (FRD) - must match model.sdf "terrain_cam" (looking straight down)
	static constexpr float kCamX = 0.60f, kCamY = 0.f, kCamZ = 0.46f;
	static constexpr int kStride = 2; ///< use every 2nd pixel in each direction
};
