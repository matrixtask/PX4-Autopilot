/****************************************************************************
 * Copyright (c) 2026 Tetra Aviation Corp. BSD-3-Clause (same terms as PX4).
 ****************************************************************************/
/**
 * @file PropellerAero.cpp
 * Advance-ratio correction for a propeller driven by gz MulticopterMotorModel.
 *
 * MulticopterMotorModel applies T = k w^2 independent of airspeed: no thrust lapse, and a slow or idling
 * propeller produces no drag. This system adds dF = T(J) - k w^2 along the propeller axis, with
 *   J = V_axial / (n D),  n = w / 2pi  (w = joint speed * slowdown, the speed the motor model uses)
 *   J <= J0 : T = k w^2 (1 - J/J0)                 (linear C_T(J), static thrust unchanged)
 *   J >  J0 : T = -q A Cd_wm * min(1, (J-J0)/J0)  (windmilling: drag rises to Cd_wm on the disk area)
 * J0 and Cd_wm are inputs (see the model file for their source); applied at the link's centre of mass.
 */
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/plugin/Register.hh>
#include <gz/common/Console.hh>

#include <cmath>

namespace tetra
{
class PropellerAero : public gz::sim::System, public gz::sim::ISystemConfigure, public gz::sim::ISystemPreUpdate
{
public:
	void Configure(const gz::sim::Entity &entity, const std::shared_ptr<const sdf::Element> &sdf,
		       gz::sim::EntityComponentManager &ecm, gz::sim::EventManager &) override
	{
		gz::sim::Model model(entity);
		_link = gz::sim::Link(model.LinkByName(ecm, sdf->Get<std::string>("link_name", "base_link").first));
		_joint = model.JointByName(ecm, sdf->Get<std::string>("joint_name", "rotor_12_joint").first);
		_axis = sdf->Get<gz::math::Vector3d>("axis", gz::math::Vector3d(1, 0, 0)).first.Normalized();
		_k = sdf->Get<double>("motor_constant", _k).first;
		_slowdown = sdf->Get<double>("rotor_velocity_slowdown_sim", _slowdown).first;
		_diameter = sdf->Get<double>("diameter", _diameter).first;
		_j0 = sdf->Get<double>("zero_thrust_advance_ratio", _j0).first;
		_cd_wm = sdf->Get<double>("windmill_cd", _cd_wm).first;
		_rho = sdf->Get<double>("air_density", _rho).first;
		_link.EnableVelocityChecks(ecm, true);

		if (_joint != gz::sim::kNullEntity && !ecm.Component<gz::sim::components::JointVelocity>(_joint)) {
			ecm.CreateComponent(_joint, gz::sim::components::JointVelocity());
		}

		_ready = _link.Valid(ecm) && _joint != gz::sim::kNullEntity;
		gzmsg << "PropellerAero: J0 " << _j0 << ", windmill Cd " << _cd_wm << (_ready ? "" : " (NOT READY)") << std::endl;
	}

	void PreUpdate(const gz::sim::UpdateInfo &info, gz::sim::EntityComponentManager &ecm) override
	{
		if (!_ready || info.paused) { return; }

		const auto v = _link.WorldLinearVelocity(ecm);
		const auto pose = _link.WorldPose(ecm);
		const auto wv = ecm.Component<gz::sim::components::JointVelocity>(_joint);

		if (!v || !pose || !wv || wv->Data().empty()) { return; }

		const gz::math::Vector3d axis_w = pose->Rot().RotateVector(_axis);
		const double v_axial = v->Dot(axis_w);              // inflow speed along the thrust axis
		const double w = std::fabs(wv->Data()[0]) * _slowdown;
		const double n = w / (2.0 * M_PI);
		const double area = M_PI * 0.25 * _diameter * _diameter;
		const double t_gz = _k * w * w;                     // what MulticopterMotorModel applies
		double t_model;

		if (v_axial <= 0.0) {
			t_model = t_gz;                                 // no forward inflow: static thrust

		} else {
			const double j = (n > 1e-3) ? v_axial / (n * _diameter) : 1e9;

			if (j <= _j0) {
				t_model = t_gz * (1.0 - j / _j0);

			} else {
				const double q = 0.5 * _rho * v_axial * v_axial;
				t_model = -q * area * _cd_wm * std::min(1.0, (j - _j0) / _j0);
			}
		}

		_link.AddWorldForce(ecm, (t_model - t_gz) * axis_w);
	}

private:
	gz::sim::Link _link{gz::sim::kNullEntity};
	gz::sim::Entity _joint{gz::sim::kNullEntity};
	gz::math::Vector3d _axis{1, 0, 0};
	double _k{0.05557}, _slowdown{10.0}, _diameter{1.8}, _j0{1.2}, _cd_wm{0.12}, _rho{1.2041};
	bool _ready{false};
};
} // namespace tetra

GZ_ADD_PLUGIN(tetra::PropellerAero, gz::sim::System, gz::sim::ISystemConfigure, gz::sim::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(tetra::PropellerAero, "PropellerAero")
