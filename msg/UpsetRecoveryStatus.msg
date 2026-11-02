# Fixed-wing upset detection and recovery (fw_att_control -> vtol_att_control, logged for analysis).

uint64 timestamp	# time since system start (microseconds)

bool active		# recovery attitude in control

uint8 TRIGGER_NONE = 0
uint8 TRIGGER_TILT = 1		# tilt above FW_UPS_TILT
uint8 TRIGGER_RATE = 2		# roll or pitch rate above FW_UPS_RT_TRG for FW_UPS_RT_T
uint8 TRIGGER_DISTURBANCE = 3	# unexplained angular acceleration above FW_UPS_D_TRG surface authorities for FW_UPS_D_T
uint8 trigger		# what started the current (or last) recovery

float32 tilt		# [rad] angle between body z and the vertical
float32[2] disturbance	# [-] roll / pitch angular acceleration not explained by the control surfaces, in units of full surface authority (high-passed)
