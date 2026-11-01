/****************************************************************************
 * Copyright (c) 2026 Tetra Aviation Corp. BSD-3-Clause (same terms as PX4).
 ****************************************************************************/
#include "FlaperonDrag.hpp"

#include <gz/plugin/Register.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/Wind.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/Util.hh>
#include <gz/common/Console.hh>

#include <cmath>
#include <sstream>

using namespace tetra;

void FlaperonDrag::Configure(const gz::sim::Entity &entity, const std::shared_ptr<const sdf::Element> &sdf,
			     gz::sim::EntityComponentManager &ecm, gz::sim::EventManager &)
{
	_model = gz::sim::Model(entity);
	_link_name = sdf->Get<std::string>("link_name", _link_name).first;
	_rho = sdf->Get<double>("air_density", _rho).first;
	_cd_plate = sdf->Get<double>("cd_plate", _cd_plate).first;
	_area = sdf->Get<double>("flap_area", _area).first;

	if (sdf->HasElement("joint_names")) {
		std::istringstream names(sdf->Get<std::string>("joint_names"));
		_joint_names.clear();

		for (std::string n; names >> n;) { _joint_names.push_back(n); }
	}

	_link = gz::sim::Link(_model.LinkByName(ecm, _link_name));
	_link.EnableVelocityChecks(ecm, true);

	for (const auto &n : _joint_names) {
		const auto j = _model.JointByName(ecm, n);

		if (j == gz::sim::kNullEntity) {
			gzerr << "FlaperonDrag: joint " << n << " not found" << std::endl;
			continue;
		}

		_joints.push_back(j);

		if (!ecm.Component<gz::sim::components::JointPosition>(j)) {
			ecm.CreateComponent(j, gz::sim::components::JointPosition());
		}
	}

	_ready = _link.Valid(ecm) && !_joints.empty();
	gzmsg << "FlaperonDrag: " << _joints.size() << " surfaces, cd " << _cd_plate << ", area " << _area << std::endl;
}

void FlaperonDrag::PreUpdate(const gz::sim::UpdateInfo &info, gz::sim::EntityComponentManager &ecm)
{
	if (!_ready || info.paused) {
		return;
	}

	const auto v_opt = _link.WorldLinearVelocity(ecm);

	if (!v_opt) {
		return;
	}

	gz::math::Vector3d v_air = *v_opt;
	const auto wind = ecm.Component<gz::sim::components::WorldLinearVelocity>(
				  ecm.EntityByComponents(gz::sim::components::Wind()));

	if (wind) {
		v_air -= wind->Data();
	}

	const double speed = v_air.Length();

	if (speed < 1.0) {
		return;
	}

	double sum_sin = 0.0;

	for (const auto j : _joints) {
		const auto pos = ecm.Component<gz::sim::components::JointPosition>(j);

		if (pos && !pos->Data().empty()) {
			sum_sin += std::sin(std::fabs(pos->Data()[0]));
		}
	}

	const double drag = 0.5 * _rho * speed * speed * _cd_plate * _area * sum_sin;
	_link.AddWorldForce(ecm, -drag * v_air / speed);
}

GZ_ADD_PLUGIN(FlaperonDrag, gz::sim::System, gz::sim::ISystemConfigure, gz::sim::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(FlaperonDrag, "FlaperonDrag")
