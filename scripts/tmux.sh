#!/usr/bin/env bash
# Open a tmux session that runs sim+teleop+webteleop together in one pane and
# an idle `hexa dev` shell in the other, both sharing the hexa-dev container.
#   ./scripts/tmux.sh            -> attach panes to the existing dev container
#   ./scripts/tmux.sh --clean    -> kill+rebuild the container first, then start
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${REPO_ROOT}"

clean=0
run_tests=0
cpp=0
split_flag="-v"
for arg in "$@"; do
    case "${arg}" in
        --clean)         clean=1 ;;
        --test)          run_tests=1 ;;
        --cpp)           cpp=1 ;;
        horizontal)      split_flag="-h" ;;
        *)
            echo "Unknown argument: ${arg}" >&2
            exit 1
            ;;
    esac
done

# Both panes drop into the dev container via `hexa dev`; forward --cpp so their
# `pod launch` / ad-hoc commands default to the C++ ports (HEXA_CPP=1).
dev_cmd="${REPO_ROOT}/hexa dev"
[[ ${cpp} -eq 1 ]] && dev_cmd="${dev_cmd} --cpp"

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

# Pane 0 (left): drop into the dev container and bring up the full sim stack
# (sim + webteleop + teleop) via `pod launch` — the same stack `hexa dev --launch` runs.
tmux new-session -d -s "${SESSION}" -n "${WINDOW}" "${dev_cmd}"
tmux send-keys -t "${SESSION}:${WINDOW}.0" "pod launch" Enter

# Pane 1 (right/below): idle dev shell for ad-hoc commands.
tmux split-window "${split_flag}" -t "${SESSION}:${WINDOW}" "${dev_cmd}"

exec tmux attach-session -t "${SESSION}"
