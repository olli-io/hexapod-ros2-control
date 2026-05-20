#!/usr/bin/env bash
# Open a tmux session that runs sim (left pane) and teleop (right pane)
# inside the shared hexa-dev container.
#   ./scripts/tmux.sh            -> attach panes to the existing dev container
#   ./scripts/tmux.sh --clean    -> kill+rebuild the container first, then start
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${REPO_ROOT}"

clean=0
run_tests=0
split_flag="-v"
for arg in "$@"; do
    case "${arg}" in
        --clean)      clean=1 ;;
        --test)       run_tests=1 ;;
        --horizontal) split_flag="-h" ;;
        *)
            echo "Unknown argument: ${arg}" >&2
            exit 1
            ;;
    esac
done

if ! command -v tmux >/dev/null 2>&1; then
    echo "Error: tmux is not installed on the host." >&2
    echo "Install it, e.g. 'sudo pacman -S tmux' on Arch or 'sudo apt install tmux' on Ubuntu." >&2
    exit 1
fi

SESSION="hexa-dev"
WINDOW="hexapod"

# With --clean, tear down any existing session so the rebuild actually happens.
if [[ ${clean} -eq 1 ]] && tmux has-session -t "${SESSION}" 2>/dev/null; then
    echo "Killing existing tmux session: ${SESSION}"
    tmux kill-session -t "${SESSION}"
else
    # Already running? Just reattach — don't spin up a second pair of panes.
    if tmux has-session -t "${SESSION}" 2>/dev/null; then
        echo "Reattaching to existing tmux session: ${SESSION}"
        exec tmux attach-session -t "${SESSION}"
    fi
fi

if [[ ${clean} -eq 1 ]]; then
    "${REPO_ROOT}/scripts/kill.sh"
    test_flag=()
    [[ ${run_tests} -eq 1 ]] && test_flag=(--test)
    "${REPO_ROOT}/scripts/dev.sh" "${test_flag[@]}" pod build
elif [[ ${run_tests} -eq 1 ]]; then
    "${REPO_ROOT}/scripts/dev.sh" --test
fi

# Make sure the container exists before both panes try to attach, so they
# don't race on the first-time `docker compose up --build`.
"${REPO_ROOT}/scripts/dev.sh" true

# Pane 0 (left): drop into the dev container and launch sim.
tmux new-session -d -s "${SESSION}" -n "${WINDOW}" "${REPO_ROOT}/hexa --dev"
tmux send-keys -t "${SESSION}:${WINDOW}.0" "sim" Enter

# Pane 1 (right/below): attach, wait for /clock, then launch teleop.
tmux split-window "${split_flag}" -t "${SESSION}:${WINDOW}" "${REPO_ROOT}/hexa --dev"
tmux send-keys -t "${SESSION}:${WINDOW}.1" \
    "echo 'waiting for sim (/clock)...'; until ros2 topic list 2>/dev/null | grep -q '^/clock\$'; do sleep 1; done; echo 'sim ready'; teleop" Enter

exec tmux attach-session -t "${SESSION}"
