/****************************************************************************
 * Copyright (c) 2026 Tetra Aviation Corp. BSD-3-Clause (same terms as PX4).
 ****************************************************************************/
#include "WakeVortex.hpp"

#include <gz/plugin/Register.hh>
#include <gz/sim/components/Wind.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/Util.hh>
#include <gz/common/Console.hh>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <sstream>

using namespace tetra;

namespace
{
constexpr double kFadeLength = 20.0; ///< [m] cos^2 fade at the ends of a vortex segment
}

void WakeVortex::Configure(const gz::sim::Entity &entity, const std::shared_ptr<const sdf::Element> &sdf,
			   gz::sim::EntityComponentManager &ecm, gz::sim::EventManager &)
{
	_model = gz::sim::Model(entity);
	_link_name = sdf->Get<std::string>("link_name", _link_name).first;
	_rho = sdf->Get<double>("air_density", _rho).first;
	_strips = std::max(2, sdf->Get<int>("strips", _strips).first);

	if (sdf->HasElement("surface")) {
		for (auto e = sdf->FindElement("surface"); e; e = e->GetNextElement("surface")) {
			std::istringstream in(e->Get<std::string>());
			Surface s;

			if (in >> s.x >> s.z >> s.span >> s.chord >> s.cla >> s.dcl_max) {
				_surfaces.push_back(s);
			}
		}
	}

	const char *env = std::getenv("MK7_WAKE");

	if (env != nullptr) {
		std::stringstream all(env);

		for (std::string entry; std::getline(all, entry, ';');) {
			std::replace(entry.begin(), entry.end(), ',', ' ');
			std::istringstream in(entry);
			double e{}, n{}, u{}, heading_deg{}, gamma{}, rc{}, b0{}, length{};

			if (!(in >> e >> n >> u >> heading_deg >> gamma >> rc >> b0 >> length)) {
				continue;
			}

			Vortex v;
			const double h = heading_deg * M_PI / 180.0;
			v.center = {e, n, u};
			v.axis = {std::sin(h), std::cos(h), 0.0};
			v.right = v.axis.Cross(gz::math::Vector3d::UnitZ);
			v.gamma = gamma;
			v.rc = std::max(rc, 0.1);
			v.b0 = b0;
			v.half_length = 0.5 * length;
			_vortices.push_back(v);
		}
	}

	_link = gz::sim::Link(_model.LinkByName(ecm, _link_name));
	_link.EnableVelocityChecks(ecm, true);
	_ready = _link.Valid(ecm) && !_surfaces.empty() && !_vortices.empty();

	gzmsg << "WakeVortex: " << _vortices.size() << " encounter(s), " << _surfaces.size() << " surface(s)"
	      << (_ready ? "" : " - inactive") << std::endl;

	for (const auto &v : _vortices) {
		gzmsg << "WakeVortex: centre " << v.center << " axis " << v.axis << " Gamma " << v.gamma << " rc " << v.rc
		      << " b0 " << v.b0 << " length " << 2.0 * v.half_length << std::endl;
	}
}

gz::math::Vector3d WakeVortex::induced(const std::vector<Vortex> &vortices, const gz::math::Vector3d &p)
{
	gz::math::Vector3d w;

	for (const auto &v : vortices) {
		const gz::math::Vector3d d = p - v.center;
		const double s = d.Dot(v.axis);
		const double outside = std::fabs(s) - (v.half_length - kFadeLength);

		if (std::fabs(s) > v.half_length) {
			continue;
		}

		const double fade = (outside > 0.0) ? std::pow(std::cos(0.5 * M_PI * outside / kFadeLength), 2) : 1.0;

		// left core +Gamma, right core -Gamma: downwash between them (generator flying along the axis)
		for (int side = -1; side <= 1; side += 2) {
			const gz::math::Vector3d core = v.center + v.right * (0.5 * v.b0 * side);
			gz::math::Vector3d r = (p - core);
			r -= v.axis * r.Dot(v.axis);
			const double r2 = r.SquaredLength();

			if (r2 < 1e-6) {
				continue;
			}

			const double gamma = (side > 0) ? -v.gamma : v.gamma;
			const double speed = gamma / (2.0 * M_PI) * std::sqrt(r2) / (r2 + v.rc * v.rc);
			w += v.axis.Cross(r.Normalized()) * (speed * fade);
		}
	}

	return w;
}

void WakeVortex::PreUpdate(const gz::sim::UpdateInfo &info, gz::sim::EntityComponentManager &ecm)
{
	if (!_ready || info.paused) {
		return;
	}

	const auto pose_opt = _link.WorldPose(ecm);
	const auto v_opt = _link.WorldLinearVelocity(ecm);

	if (!pose_opt || !v_opt) {
		return;
	}

	const gz::math::Pose3d pose = *pose_opt;
	gz::math::Vector3d v_air = *v_opt;
	const auto wind = ecm.Component<gz::sim::components::WorldLinearVelocity>(
				  ecm.EntityByComponents(gz::sim::components::Wind()));

	if (wind) {
		v_air -= wind->Data();
	}

	const double speed = v_air.Length();

	if (speed < 3.0) {
		return; // hover: the wing makes no lift worth changing (rotor inflow not modelled)
	}

	const double q = 0.5 * _rho * speed * speed;
	const gz::math::Vector3d up_body = pose.Rot().RotateVector(gz::math::Vector3d::UnitZ);
	gz::math::Vector3d force;
	gz::math::Vector3d torque; // about the link origin

	for (const auto &s : _surfaces) {
		const double dy = s.span / _strips;

		for (int i = 0; i < _strips; i++) {
			const gz::math::Vector3d p_body{s.x, -0.5 * s.span + (i + 0.5) *dy, s.z};
			const gz::math::Vector3d arm = pose.Rot().RotateVector(p_body);
			const gz::math::Vector3d w = induced(_vortices, pose.Pos() + arm);

			if (w.SquaredLength() < 1e-8) {
				continue;
			}

			// air moving up through the surface raises its angle of attack
			const double dalpha = w.Dot(up_body) / speed;
			const double dcl = std::clamp(s.cla * dalpha, -s.dcl_max, s.dcl_max);
			const gz::math::Vector3d f = up_body * (q * s.chord * dy * dcl);
			force += f;
			torque += arm.Cross(f);
		}
	}

	if (force.SquaredLength() < 1e-6) {
		return;
	}

	_link.AddWorldWrench(ecm, force, torque);

	const double t = std::chrono::duration<double>(info.simTime).count();

	if (t - _last_report > 0.5) {
		_last_report = t;
		const gz::math::Vector3d torque_body = pose.Rot().RotateVectorReverse(torque);
		gzmsg << "WakeVortex t " << t << " lift " << force.Dot(up_body) << " N  roll " << torque_body.X()
		      << " pitch " << torque_body.Y() << " yaw " << torque_body.Z() << " Nm" << std::endl;
	}
}

GZ_ADD_PLUGIN(WakeVortex, gz::sim::System, gz::sim::ISystemConfigure, gz::sim::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(WakeVortex, "WakeVortex")
