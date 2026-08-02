# Filtered, world-frame (odom) target state published by
# target_state_estimator_node. header.frame_id = the global frame (odom).
std_msgs/Header header

# True while the filter considers this estimate live enough to drive on
# (TRACKING or COASTING, in behavior_state_machine terms) -- NOT the same
# as "a real detection arrived this instant". See target_state_estimator's
# module docstring.
bool detected

# Position/velocity of the tracked person in the odom frame.
float64 x
float64 y
float64 vx
float64 vy

# 0..1, decays with age_sec since the last REAL (non-predicted) measurement.
float32 confidence

# Seconds since the last real measurement was fused, regardless of `detected`.
# This is the signal behavior_state_machine uses to pick
# TRACKING/COASTING/SEARCHING/LOST.
float32 age_sec

# 'follow' or 'flee' -- whichever tracker is currently feeding the filter.
string track_mode
