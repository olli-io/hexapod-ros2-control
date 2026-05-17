# Gait command and walk-cycle parameters.
#
# Produced by hexa_control on /gait/params at 50 Hz (last-write-wins),
# consumed by the hexa_gait engine. Carries both the commanded body
# velocity and the walk-cycle knobs (gait selection, cycle time, duty
# factor, step height). linear_z / roll / pitch belong to posture, not
# gait, and are intentionally absent.
#
# Frame: REP-103 body frame, +x forward, +y left, +z up.
# Units: linear m/s, angular rad/s, time s, length m.

std_msgs/Header header

string gait_name          # e.g. "tripod" (only one implemented in v1)

float64 linear_x          # body-frame +x velocity (forward)
float64 linear_y          # body-frame +y velocity (left)
float64 angular_z         # body-frame +z yaw rate

float64 cycle_time        # duration of one PEP -> PEP cycle
float64 duty_factor       # fraction of cycle in stance, in (0, 1)
float64 step_height       # peak swing clearance above the walk plane
