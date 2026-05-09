#!/usr/bin/env bash
exec "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/agarre_ros2_ws/scripts/limpia_stack.sh" "$@"
