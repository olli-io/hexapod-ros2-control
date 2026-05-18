# Gait command parameters.
#
# Produced by hexa_control on /gait/params at 50 Hz (last-write-wins),
# consumed by the hexa_gait engine. Carries the gait selection and the
# commanded body velocity. The walk-cycle knobs (cycle_time, duty_factor,
# step_height) live entirely in hexa_gait/config/gait.yaml and are no
# longer on the wire — cycle_time is now derived per tick from the
# commanded velocity, the configured stride_length, and duty_factor.
# linear_z / roll / pitch belong to posture, not gait.
#
# Frame: REP-103 body frame, +x forward, +y left, +z up.
# Units: linear m/s, angular rad/s.

std_msgs/Header header

string gait_name          # e.g. "tripod" (only one implemented in v1)

float64 linear_x          # body-frame +x velocity (forward)
float64 linear_y          # body-frame +y velocity (left)
float64 angular_z         # body-frame +z yaw rate
