# Body pose offset from the nominal walking pose.
#
# Origin = the body pose the gait engine would otherwise occupy at the
# commanded body velocity (body frame nominal stance: zero translation,
# zero rotation). All fields are offsets layered on top of that nominal
# pose, composed by the IK node before per-leg IK.
#
# Two intended use cases:
#   - Pose mode (gait idle, all feet grounded): yaw/x/y/z/roll/pitch
#     shift the body relative to the planted feet. The IK node holds
#     the last commanded foot positions and re-solves on every update.
#   - Gait-active body animation: the offset is composed with the foot
#     targets emitted by hexa_gait so the body can sway, lean, or bob
#     while walking. The gait strategies stay pure — they emit feet in
#     the nominal body frame; this offset rides on top.
#
# Frame: REP-103 body frame, +x forward, +y left, +z up.
# Units: metres, radians. Rotations apply about the body centre in
# the order roll → pitch → yaw (intrinsic XYZ).

std_msgs/Header header

float64 x      # translation along body +x (forward)
float64 y      # translation along body +y (left)
float64 z      # translation along body +z (up)
float64 roll   # rotation about body +x
float64 pitch  # rotation about body +y
float64 yaw    # rotation about body +z
