#!/usr/bin/env bash
# Standalone field and props; this does not start the Classic ROS robot stack.
set -euo pipefail
FIELD_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export IGN_GAZEBO_RESOURCE_PATH="${FIELD_DIR}/source/models${IGN_GAZEBO_RESOURCE_PATH:+:${IGN_GAZEBO_RESOURCE_PATH}}"
export IGN_PARTITION="${IGN_PARTITION:-scu27rc_v3_field_${USER:-user}_$$}"
exec ign gazebo -r "${FIELD_DIR}/source/initial_objects.world" \
  --gui-config "${FIELD_DIR}/source/gui/field.config" "$@"
