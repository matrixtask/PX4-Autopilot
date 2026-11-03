# In-flight identification of the multicopter control effectiveness (mc_rate_control, MC_ID_EN).
# Per axis (roll, pitch, yaw): angular acceleration = b * torque (normalized), both band-passed.
# The angular acceleration feed-forward (MC_ACC_FF_EN) uses the confirmed values MC_ID_B_R/P/Y.

uint64 timestamp	# time since system start (microseconds)

bool active		# estimator running (MC_ID_EN 1, armed, in the air, multicopter)
bool[3] converged	# enough excitation within the forgetting window to trust b_est
float32[3] b_est	# estimated effectiveness [rad/s^2 per unit torque]
float32[3] b_var	# estimator covariance P
float32[3] excitation_s	# seconds of usable excitation within the forgetting window
float32[3] b_confirmed	# MC_ID_B_R/P/Y in use by the feed-forward
float32[3] u_bp		# band-passed torque through the actuator lag (regressor)
float32[3] y_bp		# band-passed angular acceleration [rad/s^2]
bool linear		# all torques inside MC_ID_TQLIM (samples outside are not used)
