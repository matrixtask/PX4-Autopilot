# Feed-forward back-transition brake (FlightTaskTransition -> vtol_att_control, logged for analysis).
# Law: src/lib/vtol_brake/BackTransitionBrake.hpp

uint64 timestamp	# time since system start (microseconds)

float32 decel_setpoint	# [m/s^2] deceleration demand (distance-based, jerk-limited, with trim)
float32 decel		# [m/s^2] measured deceleration along the track
float32 pitch		# [rad] pitch commanded (rate-limited ideal pitch)
float32 pitch_ideal	# [rad] pitch the law aims for
float32 spoiler		# [0, 1] spoiler command
float32 lift		# [N] predicted wing lift (fed forward to the rotors)
float32 sink_max	# [m/s] sink rate that still leaves the angle of attack to brake
float32 airspeed	# [m/s] speed the law used (calibrated airspeed, or ground speed without airspeed)
