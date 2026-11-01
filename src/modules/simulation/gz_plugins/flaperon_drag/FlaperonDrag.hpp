/****************************************************************************
 * Copyright (c) 2026 Tetra Aviation Corp. BSD-3-Clause (same terms as PX4).
 ****************************************************************************/
/**
 * @file FlaperonDrag.hpp
 * Drag of deflected flaperons / spoilers, missing from gz LiftDrag (which only changes CL with deflection).
 *
 * Per surface: D_i = q * cd_plate * flap_area * sin(|delta_i|), applied at the link's centre of mass against the
 * air-relative velocity (symmetric deflection = pure drag, no rolling moment). q = 0.5 rho |v|^2.
 * Matches the [MOD-4] term documented in the Mk-7 model (teTra_mk-7_EM2/EM3); the original binary was not in any
 * repository. `baseline_rad` from that description is accepted and ignored (its meaning is not documented).
 * Optional `base_cda` [m^2]: constant parasite drag of the airframe parts the lift models do not cover (booms,
 * stopped lift rotors, landing gear), D = q * base_cda, same point and direction.
 */
#pragma once

#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Joint.hh>

#include <string>
#include <vector>

namespace tetra
{
class FlaperonDrag : public gz::sim::System, public gz::sim::ISystemConfigure, public gz::sim::ISystemPreUpdate
{
public:
	void Configure(const gz::sim::Entity &entity, const std::shared_ptr<const sdf::Element> &sdf,
		       gz::sim::EntityComponentManager &ecm, gz::sim::EventManager &events) override;
	void PreUpdate(const gz::sim::UpdateInfo &info, gz::sim::EntityComponentManager &ecm) override;

private:
	gz::sim::Model _model{gz::sim::kNullEntity};
	gz::sim::Link _link{gz::sim::kNullEntity};
	std::vector<gz::sim::Entity> _joints;
	std::vector<std::string> _joint_names{"servo_5", "servo_6"};
	std::string _link_name{"base_link"};
	double _rho{1.2041};
	double _cd_plate{1.0};
	double _area{0.9};
	double _base_cda{0.0};  ///< constant airframe parasite drag area [m^2] (booms, stopped lift rotors, gear)
	bool _ready{false};
};
} // namespace tetra
