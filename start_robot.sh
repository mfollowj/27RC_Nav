#!/usr/bin/env bash
SCRIPT_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
ROOT="$(cd "$(dirname "${SCRIPT_PATH}")" && pwd)"
exec bash "${ROOT}/robot/start_robot.sh" "$@"
