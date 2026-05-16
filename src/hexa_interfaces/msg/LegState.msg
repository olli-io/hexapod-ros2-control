# Per-leg foot target plus gait-state metadata.
#
# Produced by hexa_gait, consumed by hexa_kinematics. The IK node only
# needs foot_target (plus the leg name to look up the LegSpec); phase
# and stance are informational, kept here so downstream subscribers
# (loggers, body animations, debug tooling) get a single coherent
# snapshot per tick.
#
# Frame: REP-103 body frame (+x forward, +y left, +z up). The target
# is the foot position in the *nominal* body frame — i.e. before
# /body/pose_target is composed in. The IK node does that composition.
#
# Naming follows hexa_description's URDF: leg_name is one of
# {l_front, l_middle, l_rear, r_front, r_middle, r_rear}. Use these
# exact strings; the IK node dispatches on them.

string leg_name
geometry_msgs/Point foot_target  # metres
float64 phase                    # gait phase in [0, 1), 0 at lift-off
bool stance                      # true = foot on ground, false = swing
