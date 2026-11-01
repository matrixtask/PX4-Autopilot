# Ground surface under the vehicle for landing, from a downward terrain camera (depth) plane fit.
# SITL: gz_bridge (depth camera). Hardware: a companion computer publishing the same fields.
# Fit and gear clearance: src/lib/landing_surface/PlaneFit.hpp

uint64 timestamp		# time since system start (microseconds)
uint64 timestamp_sample		# image time

bool valid			# plane found with enough inliers
float32 height			# [m] vertical distance from the vehicle origin down to the plane (z = dzdx x + dzdy y + height)
float32 dzdx			# [-] plane gradient towards north (z down, level NED frame centred on the vehicle)
float32 dzdy			# [-] plane gradient towards east
float32 slope			# [rad] plane inclination from horizontal
float32 slope_azimuth		# [rad] direction of steepest descent, NED heading
float32 roughness		# [m] RMS residual of the inliers
uint16 inliers			# points used
uint16 points			# valid depth points (inliers / points < 1: obstacles or a step in view)
