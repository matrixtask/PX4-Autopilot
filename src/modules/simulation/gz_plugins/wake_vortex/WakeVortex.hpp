/****************************************************************************
 * Copyright (c) 2026 Tetra Aviation Corp. BSD-3-Clause (same terms as PX4).
 ****************************************************************************/
/**
 * @file WakeVortex.hpp
 * Wake-vortex encounter: the span-wise flow of a trailing vortex pair, which the uniform-flow lift models
 * (LiftDrag / AdvancedLiftDrag) cannot represent. Each lifting surface is split into strips; the vortex-induced
 * velocity normal to the surface changes the local angle of attack, the extra lift dL = q c dy CLa da (limited to
 * +-dcl_max, a crude stall) is applied at the strip, so one wing in the upwash and the other in the downwash gives
 * the rolling moment of a wake encounter.
 *
 * Vortex pair (Burnham-Hallock): v_theta(r) = Gamma / (2 pi) * r / (r^2 + rc^2), cores b0 apart, downwash between
 * them, along a horizontal axis over a finite length (cos^2 fade over the last 20 m at each end). Fixed in the
 * world (no sink, no decay).
 *
 * Configuration: SDF <surface>x z span chord cla dcl_max</surface> (body frame FLU [m], repeatable) and the
 * environment variable MK7_WAKE, encounters separated by ';':
 *   east,north,up,heading_deg,gamma,rc,b0,length   (world ENU [m], axis heading from north clockwise [deg],
 *                                                   circulation [m^2/s], core radius [m], core spacing [m], length [m])
 * Without MK7_WAKE the plugin does nothing.
 *
 * Not modelled: the induced flow on the lift rotors and the pusher, the vortex descent and decay, the fuselage.
 */
#pragma once

#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/math/Vector3.hh>

#include <string>
#include <vector>

namespace tetra
{
class WakeVortex : public gz::sim::System, public gz::sim::ISystemConfigure, public gz::sim::ISystemPreUpdate
{
public:
	void Configure(const gz::sim::Entity &entity, const std::shared_ptr<const sdf::Element> &sdf,
		       gz::sim::EntityComponentManager &ecm, gz::sim::EventManager &events) override;
	void PreUpdate(const gz::sim::UpdateInfo &info, gz::sim::EntityComponentManager &ecm) override;

	struct Vortex {
		gz::math::Vector3d center;  ///< world ENU [m]
		gz::math::Vector3d axis;    ///< unit, horizontal
		gz::math::Vector3d right;   ///< unit, horizontal, axis x up
		double gamma{0.0};          ///< [m^2/s]
		double rc{1.0};             ///< core radius [m]
		double b0{10.0};            ///< core spacing [m]
		double half_length{500.0};  ///< [m]
	};

	struct Surface {
		double x{0.0};       ///< body FLU [m]
		double z{0.0};
		double span{1.0};
		double chord{1.0};
		double cla{4.7};     ///< [1/rad]
		double dcl_max{1.0}; ///< |dCL| limit (stall)
	};

	/** Induced velocity [m/s] of all vortex pairs at a world point. */
	static gz::math::Vector3d induced(const std::vector<Vortex> &vortices, const gz::math::Vector3d &p);

private:
	gz::sim::Model _model{gz::sim::kNullEntity};
	gz::sim::Link _link{gz::sim::kNullEntity};
	std::string _link_name{"base_link"};
	double _rho{1.2041};
	int _strips{16};
	std::vector<Surface> _surfaces;
	std::vector<Vortex> _vortices;
	double _last_report{-1.0};
	bool _ready{false};
};
} // namespace tetra
