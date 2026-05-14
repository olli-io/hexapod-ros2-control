# Twist and `cmd_vel`

`Twist` is the message type. `cmd_vel` is the topic. Together they are the
ROS-standard way to command an instantaneous robot velocity. This doc
explains what the message means mathematically.

## 1. Velocity of a single point

A point in 3D space has a position vector:

```
p = (x, y, z)
```

Its velocity is the time derivative:

```
v = dp/dt = (ẋ, ẏ, ż)        units: m/s
```

## 2. A rigid body also rotates

A hexapod body is not a point — it can also spin. In 2D, "spin rate" is one
number ω (rad/s) because there's only one possible axis (out of the page). In
3D, you can spin around any axis, so angular velocity is itself a vector:

```
ω = (ω_x, ω_y, ω_z)          units: rad/s
```

How to read it:

- **Direction** of ω = the axis the body is rotating about.
- **Magnitude** |ω| = how fast (rad/s).
- **Sign convention**: right-hand rule. Point your right thumb along ω; your
  fingers curl in the direction of rotation.

Example: `ω = (0, 0, π)` means spinning about the z-axis at π rad/s (half a
revolution per second), counter-clockwise when viewed from above.

## 3. Six numbers describe a rigid body's full motion

At any instant, a rigid body's motion is completely determined by:

- Linear velocity of its origin: `v = (v_x, v_y, v_z)`  (3 numbers)
- Angular velocity:               `ω = (ω_x, ω_y, ω_z)` (3 numbers)

That's 6 degrees of freedom. This is exactly what `geometry_msgs/Twist`
carries:

```
linear:  Vector3    # (v_x, v_y, v_z)   in m/s
angular: Vector3    # (ω_x, ω_y, ω_z)   in rad/s
```

A Twist is one mathematical object: the instantaneous 6-DOF velocity of a
rigid body.

## 4. The fundamental formula

Given a Twist (v, ω) for the body, the velocity of *any* point r on the body
(measured from the body's origin) is:

```
ṙ = v + ω × r
```

where `×` is the vector cross product.

**Hexapod example.** Suppose the body is spinning in place: `v = 0`, `ω = (0, 0, 1)` rad/s.
A foot anchored to the body at `r = (0.2, 0, 0)` (20 cm forward of body origin)
has world-frame velocity:

```
ṙ = 0 + (0,0,1) × (0.2,0,0) = (0, 0.2, 0)
```
0.2 m/s in the +y direction (to the left, see §5).

## 5. Frame and axis conventions (REP-103)

ROS uses a right-handed body frame:

- `x` — forward
- `y` — left
- `z` — up

A `Twist` on `cmd_vel` is interpreted in the robot's body frame: "+x forward"
means forward *from the robot's point of view*, regardless of where the robot
is facing in the world.

For a hexapod walking on flat ground, the components that matter:

- `linear.x` — forward speed; walk forward (+) or backward (−)
- `linear.y` — strafe speed; sidestep left (+) or right (−)
- `linear.z` — vertical speed; usually 0 on flat ground
- `angular.x` — roll rate; usually 0
- `angular.y` — pitch rate; usually 0
- `angular.z` — yaw rate; turn left (+) or right (−)


## 6. Concrete examples

Each entry below is `linear` / `angular` → behaviour:

- `(0, 0, 0)` / `(0, 0, 0)` — Stand still
- `(0.1, 0, 0)` / `(0, 0, 0)` — Walk forward at 0.1 m/s
- `(0, 0, 0)` / `(0, 0, 0.5)` — Spin in place, 0.5 rad/s counter-clockwise
- `(0.1, 0, 0)` / `(0, 0, 0.5)` — Walk forward while turning (curved path)
- `(0, 0.1, 0)` / `(0, 0, 0)` — Sidestep left at 0.1 m/s
- `(-0.05, 0, 0)` / `(0, 0, -0.3)` — Reverse and turn right

## 7. Summary

- A Twist is **one** object: the instantaneous 6-DOF velocity of a rigid body.
- 3 linear components (m/s) + 3 angular components (rad/s) = 6 DOF.
- Interpreted in the **body frame** (REP-103: x forward, y left, z up).
- The formula `ṙ = v + ω × r` turns a body Twist into a velocity for every
  point on the body — including each foot.
- The name comes from screw theory: every rigid-body motion is a screw
  (rotation about an axis + translation along it), and a Twist captures that
  screw rate in exactly 6 numbers — not more, not less.
- `cmd_vel` is the ROS-wide topic convention; speaking it makes the stack
  plug-compatible with `teleop_twist_*`, `twist_mux`, Nav2, and sim plugins
  without adapter code.
