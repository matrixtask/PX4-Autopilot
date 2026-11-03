# Online identification of the multicopter yaw control effectiveness (mc_rate_control, MC_YAW_ID_EN).
# Model: yaw angular acceleration = b * yaw torque (normalized), both band-passed; the yaw acceleration feed-forward
# MC_YAW_ACC_FF is 1 / b.

uint64 timestamp	# time since system start (microseconds)

bool active		# estimator running (armed, in the air, multicopter)
bool converged		# enough excitation within the forgetting window to trust b_est
float32 b_est		# estimated yaw effectiveness [rad/s^2 per unit torque]
float32 b_var		# estimator covariance P
float32 excitation_s	# seconds of usable excitation within the forgetting window
float32 ff_est		# 1 / b_est, limited [unit torque per rad/s^2]
float32 ff_applied	# feed-forward gain in use
float32 u_bp		# band-passed yaw torque (lagged), regressor
float32 y_bp		# band-passed yaw angular acceleration [rad/s^2]
