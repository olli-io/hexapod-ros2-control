# Per-tick foot targets for all six legs.
#
# Published by hexa_gait on /legs/targets, consumed by the hexa_kinematics
# IK node. The fixed-size legs[6] array carries one LegState per leg; the
# IK node dispatches by LegState.leg_name so producers don't have to
# commit to a positional order.

std_msgs/Header header
LegState[6] legs

# Master phase in [0, 1) from the gait clock. phase = 0 is lift-off for
# the reference leg (the one with phase_offset = 0). Consumers that
# need a phase signal that is shared across legs (e.g. posture
# animations synced to the gait) read this instead of reconstructing
# it from LegState[].phase + offsets.
float64 master_phase
