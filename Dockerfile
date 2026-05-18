# syntax=docker/dockerfile:1.7
FROM osrf/ros:jazzy-desktop

ARG USER_UID=1000
ARG USER_GID=1000
ARG USERNAME=dev

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-ros-gz \
        ros-jazzy-ros2-control \
        ros-jazzy-ros2-controllers \
        ros-jazzy-controller-manager \
        ros-jazzy-gz-ros2-control \
        ros-jazzy-xacro \
        ros-jazzy-joint-state-publisher-gui \
        python3-colcon-common-extensions \
        python3-vcstool \
        build-essential \
        git \
        vim \
        less \
        sudo \
        usbutils \
    && rm -rf /var/lib/apt/lists/*

# Ubuntu 24.04 ships a default `ubuntu` user/group at UID/GID 1000. Rename
# it to ${USERNAME} (preserving the UID) so bind-mounted files stay owned by
# the host user; fall back to fresh creation if the base image ever changes.
RUN if id ubuntu >/dev/null 2>&1; then \
        groupmod --new-name ${USERNAME} ubuntu \
        && usermod --login ${USERNAME} --move-home --home /home/${USERNAME} ubuntu \
        && usermod --uid ${USER_UID} ${USERNAME} \
        && groupmod --gid ${USER_GID} ${USERNAME} ; \
    else \
        groupadd --gid ${USER_GID} ${USERNAME} \
        && useradd --uid ${USER_UID} --gid ${USER_GID} --create-home --shell /bin/bash ${USERNAME} ; \
    fi \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

COPY docker/entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

USER ${USERNAME}
WORKDIR /workspace

# Put the workspace root on PATH so `hexa` is callable from every shell type
# (interactive, non-interactive, `docker exec container hexa build`, ...).
ENV PATH="/workspace:${PATH}"

RUN echo "source /opt/ros/jazzy/setup.bash" >> /home/${USERNAME}/.bashrc \
    && echo '[ -f /workspace/install/setup.bash ] && source /workspace/install/setup.bash' >> /home/${USERNAME}/.bashrc \
    && echo '[ -f /workspace/docker/aliases.sh ] && source /workspace/docker/aliases.sh' >> /home/${USERNAME}/.bashrc

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["bash"]
