#!/usr/bin/env bash
# Entry point for hexapod project commands.
# Dispatches to per-process scripts under scripts/.
#   ./hexapod.sh --dev [args...]   -> drop into the ROS2 Jazzy dev container
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

usage() {
    cat <<EOF
Usage: ./hexapod.sh <command> [args...]

Commands:
  --dev [args...]   Drop into the ROS2 Jazzy dev container (forwards args to the container).
  kill              Stop and remove the dev container.
  -h, --help        Show this help message.
EOF
}

if [[ $# -lt 1 ]]; then
    usage
    exit 1
fi

cmd="$1"
shift

case "${cmd}" in
    --dev)
        exec "${REPO_ROOT}/scripts/dev.sh" "$@"
        ;;
    kill)
        exec "${REPO_ROOT}/scripts/kill.sh" "$@"
        ;;
    -h|--help)
        usage
        ;;
    *)
        echo "Unknown command: ${cmd}" >&2
        usage
        exit 1
        ;;
esac
