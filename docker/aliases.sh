# Shell init sourced inside the dev container.
# Edit freely — no image rebuild required.

# Put the workspace root on PATH so `hexa` is callable from anywhere.
export PATH="/workspace:${PATH}"

alias cb='colcon build --symlink-install'
alias sim='ros2 launch hexa_bringup sim.launch.py'
alias teleop='ros2 launch hexa_teleop teleop.launch.py'
