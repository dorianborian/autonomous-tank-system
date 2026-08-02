# behavior_state_machine_node telemetry. Published latched (transient_local)
# since it is persistent state, same rationale as the drive_source/track_mode
# topics in mode_constants.py.
std_msgs/Header header

# One of: TRACKING | COASTING | SEARCHING | RECOVERING | LOST
string state

# Short human-readable reason, for the web UI event log (e.g. "left frame,
# rotating to last bearing 0.62" or "stuck detected, backing up").
string reason
