#!/usr/bin/env bash
# Tear down the hexapod ROS2 Jazzy dev container.
# No-op if the container doesn't exist.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${REPO_ROOT}"

CONTAINER_NAME="hexapod-dev"

if [ -z "$(docker ps -aq --filter "name=^${CONTAINER_NAME}$")" ]; then
    echo "No ${CONTAINER_NAME} container to kill."
    exit 0
fi

docker rm -f "${CONTAINER_NAME}" >/dev/null
echo "Removed ${CONTAINER_NAME}."
