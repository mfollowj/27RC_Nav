#!/usr/bin/env bash
# Fortress BR/TR physics scene; no ROS bridge, LIO or Nav2 is started here.
set -euo pipefail
FIELD_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
SIM_DIR="$(cd -- "${FIELD_DIR}/../.." && pwd)"
export IGN_GAZEBO_RESOURCE_PATH="${FIELD_DIR}/source/models:${SIM_DIR}/robots/models${IGN_GAZEBO_RESOURCE_PATH:+:${IGN_GAZEBO_RESOURCE_PATH}}"
export IGN_PARTITION="${IGN_PARTITION:-scu27rc_v3_robots_${USER:-user}_$$}"
exec ign gazebo -r "${FIELD_DIR}/worlds/robocon_cn_v3_draft_tr_br.world" \
  --gui-config "${FIELD_DIR}/source/gui/field.config" "$@"
