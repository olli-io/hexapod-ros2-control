# Per-tick foot targets for all six legs.
#
# Published by hexa_gait on /legs/targets, consumed by the hexa_kinematics
# IK node. The fixed-size legs[6] array carries one LegState per leg; the
# IK node dispatches by LegState.leg_name so producers don't have to
# commit to a positional order.

std_msgs/Header header
LegState[6] legs
