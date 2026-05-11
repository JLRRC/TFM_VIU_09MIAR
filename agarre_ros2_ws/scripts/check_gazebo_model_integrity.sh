#!/usr/bin/env bash
# Ruta/archivo: agarre_ros2_ws/scripts/check_gazebo_model_integrity.sh
# Resumen: valida con el stack vivo que Gazebo cargue UN solo UR5+RG2 coherente
#          (sin duplicados, sin links sueltos, con TF + /joint_states + ros2_control
#          activos). Si todo OK, escribe /tmp/gazebo_model_integrity_ok como
#          flag que el gate de pick consume.
# Uso:    ./check_gazebo_model_integrity.sh
# Salida: exit 0 + flag escrito; exit 1 + flag borrado.
#
# Pre-requisitos:
#   - Stack arrancado (ur5_stack.launch.py).
#   - GZ_PARTITION exportada o en log/gz_partition.txt.
#   - ros2 sourced (`source install/setup.bash`).

set -u

WS_DIR="${WS_DIR:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
LOG_DIR="${LOG_DIR:-${WS_DIR}/log}"
PART_FILE="${LOG_DIR}/gz_partition.txt"
WORLD_NAME="${WORLD_NAME:-ur5_mesa_objetos}"
FLAG_FILE="${GAZEBO_INTEGRITY_FLAG:-/tmp/gazebo_model_integrity_ok}"
POSE_FILE="${POSE_INFO_DUMP:-/tmp/gz_pose_info.txt}"

if [ -z "${GZ_PARTITION:-}" ] && [ -f "${PART_FILE}" ]; then
    GZ_PARTITION="$(cat "${PART_FILE}")"
    export GZ_PARTITION
fi

FAIL=0
PASS_COUNT=0
FAIL_COUNT=0

note() { printf '[CHK]  %s\n' "$*"; }
ok()   { PASS_COUNT=$((PASS_COUNT + 1)); printf '[PASS] %s\n' "$*"; }
bad()  { FAIL_COUNT=$((FAIL_COUNT + 1)); FAIL=1; printf '[FAIL] %s\n' "$*" >&2; }
warn() { printf '[WARN] %s\n' "$*"; }

require_cmd() {
    if ! command -v "$1" >/dev/null 2>&1; then
        bad "comando requerido no encontrado: $1"
        rm -f "${FLAG_FILE}"
        echo "GAZEBO_MODEL_INTEGRITY_OK=false"
        exit 2
    fi
}

# Borrado preventivo del flag — solo lo escribimos al final si todo pasa
rm -f "${FLAG_FILE}"

require_cmd gz
require_cmd ros2
require_cmd awk
require_cmd grep

# ----------------------------------------------------------------------------
# (1) GZ_PARTITION / Gazebo vivo
# ----------------------------------------------------------------------------
note "Gazebo vivo (GZ_PARTITION='${GZ_PARTITION:-<unset>}')"
GZ_MODELS_OUT="$(timeout 8 gz model --list 2>/dev/null || true)"
if [ -z "${GZ_MODELS_OUT}" ]; then
    bad "gz model --list no devolvio nada (Gazebo no responde o GZ_PARTITION mal)"
    GZ_AVAILABLE=0
else
    ok "Gazebo responde a 'gz model --list'"
    GZ_AVAILABLE=1
fi

# ----------------------------------------------------------------------------
# (3) UN solo modelo top-level ur5_rg2
# ----------------------------------------------------------------------------
if [ "${GZ_AVAILABLE}" -eq 1 ]; then
    UR5_COUNT="$(printf '%s\n' "${GZ_MODELS_OUT}" | awk '/[[:space:]]ur5_rg2$|^ur5_rg2$/{c++} END{print c+0}')"
    if [ "${UR5_COUNT}" -eq 1 ]; then
        ok "one top-level ur5_rg2 model"
    elif [ "${UR5_COUNT}" -eq 0 ]; then
        bad "modelo ur5_rg2 NO esta en gz model --list"
    else
        bad "ur5_rg2 DUPLICADO en gz model --list (${UR5_COUNT} veces)"
    fi
fi

# ----------------------------------------------------------------------------
# (4) NO hay links sueltos como modelos top-level
# ----------------------------------------------------------------------------
if [ "${GZ_AVAILABLE}" -eq 1 ]; then
    STRAYS_RE='^(rg2|rg2_base_link|rg2_finger_link1|rg2_finger_link2|finger|wrist_1_link|wrist_2_link|wrist_3_link|tool0|flange|shoulder_link|upper_arm_link|forearm_link)$'
    FOUND_STRAYS="$(printf '%s\n' "${GZ_MODELS_OUT}" | awk '{for(i=1;i<=NF;i++) print $i}' | grep -E "${STRAYS_RE}" | sort -u || true)"
    if [ -z "${FOUND_STRAYS}" ]; then
        ok "no floating rg2/finger/wrist models"
    else
        bad "links del robot aparecen como modelos top-level: $(printf '%s\n' "${FOUND_STRAYS}" | tr '\n' ' ')"
    fi
fi

# ----------------------------------------------------------------------------
# (5)(6) pose/info topic + dump
# ----------------------------------------------------------------------------
POSE_TOPIC="/world/${WORLD_NAME}/pose/info"
if [ "${GZ_AVAILABLE}" -eq 1 ]; then
    if gz topic -l 2>/dev/null | grep -qE "^${POSE_TOPIC}\$"; then
        ok "topic ${POSE_TOPIC} expuesto"
        if timeout 5 gz topic -e -n 1 -t "${POSE_TOPIC}" > "${POSE_FILE}" 2>/dev/null && [ -s "${POSE_FILE}" ]; then
            ok "pose/info capturado en ${POSE_FILE}"
            if grep -q 'name: "ur5_rg2"' "${POSE_FILE}"; then
                ok "pose/info contiene ur5_rg2"
            else
                bad "pose/info NO contiene ur5_rg2"
            fi
        else
            bad "no se pudo capturar mensaje de ${POSE_TOPIC}"
        fi
    else
        bad "topic ${POSE_TOPIC} ausente"
    fi
fi

# ----------------------------------------------------------------------------
# (7) TF base_link -> {tool0, rg2_base_link, rg2_pinch_center, rg2_finger_*}
# ----------------------------------------------------------------------------
tf_xyz() {
    local parent="$1" child="$2"
    timeout 4 ros2 run tf2_ros tf2_echo "${parent}" "${child}" 2>/dev/null \
        | awk '/Translation:/ {gsub(/[][,]/, ""); print $2, $3, $4; exit}'
}

dist3d() {
    awk -v s="$1" 'BEGIN{n=split(s,a," "); x=a[1]; y=a[2]; z=a[3]; print sqrt(x*x+y*y+z*z)}'
}

declare -A TF_XYZ
for child in tool0 rg2_base_link rg2_pinch_center rg2_finger_link1 rg2_finger_link2; do
    note "TF base_link -> ${child}"
    val="$(tf_xyz base_link "${child}")"
    if [ -z "${val}" ]; then
        bad "TF base_link -> ${child} no disponible"
        TF_XYZ[${child}]=""
    else
        ok "TF base_link -> ${child} = (${val})"
        TF_XYZ[${child}]="${val}"
    fi
done

# ----------------------------------------------------------------------------
# (8) Distancias internas RG2 dentro de tolerancia
# ----------------------------------------------------------------------------
check_dist() {
    local parent="$1" child="$2" tol="$3"
    local val
    val="$(tf_xyz "${parent}" "${child}")"
    if [ -z "${val}" ]; then
        bad "TF ${parent} -> ${child} no disponible (necesario para distancia)"
        return
    fi
    local d
    d="$(dist3d "${val}")"
    if awk -v d="${d}" -v t="${tol}" 'BEGIN{exit (d < t) ? 0 : 1}'; then
        ok "distance ${parent} -> ${child} = ${d} m (< ${tol})"
    else
        bad "distance ${parent} -> ${child} = ${d} m EXCEDE ${tol} m"
    fi
}

note "distancias internas RG2"
check_dist tool0            rg2_base_link    0.30
check_dist rg2_base_link    rg2_finger_link1 0.20
check_dist rg2_base_link    rg2_finger_link2 0.20
check_dist rg2_finger_link1 rg2_finger_link2 0.20

# ----------------------------------------------------------------------------
# (9) /joint_states con UR5 + RG2
# ----------------------------------------------------------------------------
note "/joint_states: UR5 + RG2 joints"
JS_DUMP="$(timeout 5 ros2 topic echo --once /joint_states 2>/dev/null || true)"
if [ -z "${JS_DUMP}" ]; then
    bad "/joint_states no esta publicando"
else
    EXPECTED_ARM=(shoulder_pan_joint shoulder_lift_joint elbow_joint wrist_1_joint wrist_2_joint wrist_3_joint)
    MISS_ARM=()
    for j in "${EXPECTED_ARM[@]}"; do
        if ! printf '%s' "${JS_DUMP}" | grep -qF "${j}"; then
            MISS_ARM+=("${j}")
        fi
    done
    if [ "${#MISS_ARM[@]}" -eq 0 ]; then
        ok "/joint_states contiene los 6 joints del UR5"
    else
        bad "/joint_states faltan joints del UR5: ${MISS_ARM[*]}"
    fi
    if printf '%s' "${JS_DUMP}" | grep -qE "rg2_finger_joint1|rg2_finger_joint2"; then
        ok "/joint_states contiene gripper joints (RG2)"
    else
        warn "/joint_states no expone rg2_finger_joint1/2 (revisar gripper controller)"
    fi
fi

# ----------------------------------------------------------------------------
# (10) ros2 control list_controllers: state_broadcaster + arm + gripper
# ----------------------------------------------------------------------------
note "ros2 control list_controllers"
CTRL_OUT="$(timeout 8 ros2 control list_controllers 2>/dev/null || true)"
if [ -z "${CTRL_OUT}" ]; then
    bad "ros2 control list_controllers no devuelve nada (controller_manager?)"
else
    if printf '%s' "${CTRL_OUT}" | grep -qE "joint_state_broadcaster[[:space:]]+.*[[:space:]](active|configured)"; then
        ok "joint_state_broadcaster activo"
    else
        bad "joint_state_broadcaster NO activo"
    fi
    if printf '%s' "${CTRL_OUT}" | grep -qE "(joint_trajectory_controller|scaled_joint_trajectory_controller|arm_controller)[[:space:]].*active"; then
        ok "controlador del brazo activo"
    else
        bad "controlador del brazo NO encontrado activo"
    fi
    if printf '%s' "${CTRL_OUT}" | grep -qE "(rg2|gripper)[^[:space:]]*[[:space:]].*active"; then
        ok "controlador del gripper activo"
    else
        warn "no se detecta controlador de gripper activo (puede ser opcional)"
    fi
fi

# ----------------------------------------------------------------------------
# Resumen
# ----------------------------------------------------------------------------
echo "---"
printf 'PASS=%d  FAIL=%d\n' "${PASS_COUNT}" "${FAIL_COUNT}"
if [ "${FAIL}" -eq 0 ]; then
    date -u +"GAZEBO_INTEGRITY_OK_AT=%Y-%m-%dT%H:%M:%SZ" > "${FLAG_FILE}"
    printf 'GAZEBO_MODEL_INTEGRITY_OK=true\n'
    printf 'flag: %s\n' "${FLAG_FILE}"
    exit 0
else
    rm -f "${FLAG_FILE}"
    printf 'GAZEBO_MODEL_INTEGRITY_OK=false\n'
    exit 1
fi
