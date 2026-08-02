# Single-frame, timestamped raw tracker detection. header.stamp is the
# CAPTURE time of the source image (propagated from the camera driver's
# Image message), not the time this detection was published -- this is what
# lets target_state_estimator do a proper tf2 time-travel TF lookup instead
# of fusing against whatever pose the robot happens to have when the
# detection message is processed.
std_msgs/Header header
bool detected
float32 bearing
float32 distance
