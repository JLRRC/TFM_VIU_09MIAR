#!/usr/bin/env bash
# Ruta/archivo: agarre_ros2_ws/scripts/check_ur5_rg2_sdf_convention.sh
# Resumen: chequeo estatico que valida que model.sdf use el convenio SDF
#          original ros-industrial (ejes Y para shoulder_lift/elbow/wrist_1/3,
#          eje Z para shoulder_pan/wrist_2) y NO la migracion rota a DH/axis-Z
#          que rompe las meshes visuales del UR5.
# Uso:    ./check_ur5_rg2_sdf_convention.sh
# Salida: exit 0 si OK, exit 1 si detecta convenio DH/URDF roto.
#
# Contexto:
#   - HEAD (correcto) usa el convenio del paquete ros-industrial:
#       shoulder_pan   pose "0 0 0.089159 0 0 0"        axis "0 0 1"
#       shoulder_lift  pose "0 0.13585 0 0 0 0"          axis "0 1 0"
#       elbow          pose "0 -0.1197 0.425 0 0 0"      axis "0 1 0"
#       wrist_1        pose "0 0 0.39225 0 0 0"          axis "0 1 0"
#       wrist_2        pose "0 0.093 0 0 0 0"            axis "0 0 1"
#       wrist_3        pose "0 0 0.09465 0 0 0"          axis "0 1 0"
#   - Migracion rota (DH-style) mete axis "0 0 1" en TODOS los joints y poses
#     tipo "-0.425 0 0" / "-0.39225 0 0.10915" que dejan las meshes desencajadas.

set -u

WS_DIR="${WS_DIR:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
SDF_PATH="${SDF_PATH:-${WS_DIR}/src/ur5_gazebo/models/ur5_rg2/model.sdf}"

FAIL=0
PASS=0

note() { printf '[CHK]  %s\n' "$*"; }
ok()   { PASS=$((PASS + 1)); printf '[PASS] %s\n' "$*"; }
bad()  { FAIL=$((FAIL + 1)); printf '[FAIL] %s\n' "$*" >&2; }

if [ ! -f "${SDF_PATH}" ]; then
    bad "model.sdf no encontrado: ${SDF_PATH}"
    echo "GAZEBO_SDF_CONVENTION_OK=false"
    exit 1
fi
note "modelo: ${SDF_PATH}"

# Extrae para un joint la pose+xyz como dos lineas:
#   pose: <texto>
#   xyz: <texto>
extract_joint_block() {
    local joint="$1"
    awk -v j="${joint}" '
        $0 ~ "<joint name=\"" j "\"" { inj=1; next }
        inj && /<pose / {
            line=$0; sub(/^[[:space:]]+/,"",line); print "pose:" line
        }
        inj && /<xyz>/ {
            line=$0; sub(/^[[:space:]]+/,"",line); print "xyz:" line
        }
        inj && /<\/joint>/ { inj=0 }
    ' "${SDF_PATH}"
}

# Asserts: el bloque del joint contiene ambas substrings esperadas (pose+axis).
check_joint() {
    local joint="$1" expect_axis="$2" expect_pose_substr="$3"
    local block axis_line pose_line
    block="$(extract_joint_block "${joint}")"
    pose_line="$(printf '%s\n' "${block}" | grep '^pose:' | head -1)"
    axis_line="$(printf '%s\n' "${block}" | grep '^xyz:'  | head -1)"

    if [ -z "${pose_line}" ] || [ -z "${axis_line}" ]; then
        bad "${joint}: bloque incompleto (pose='${pose_line}' axis='${axis_line}')"
        return
    fi

    if printf '%s' "${pose_line}" | grep -qF "${expect_pose_substr}"; then
        ok  "${joint}: pose contiene '${expect_pose_substr}'"
    else
        bad "${joint}: pose NO contiene '${expect_pose_substr}' (actual: ${pose_line})"
    fi

    if printf '%s' "${axis_line}" | grep -qE "<xyz>[[:space:]]*${expect_axis}[[:space:]]*</xyz>"; then
        ok  "${joint}: axis ${expect_axis}"
    else
        bad "${joint}: axis NO es '${expect_axis}' (actual: ${axis_line})"
    fi
}

note "shoulder_pan_joint (eje Z vertical, pose con 0.089159 en Z)"
check_joint "shoulder_pan_joint"  "0 0 1" "0 0 0.089159"

note "shoulder_lift_joint (eje Y, pose con 0.13585 en Y)"
check_joint "shoulder_lift_joint" "0 1 0" "0 0.13585 0"

note "elbow_joint (eje Y, pose con -0.1197 en Y y 0.425 en Z)"
check_joint "elbow_joint"         "0 1 0" "0 -0.1197 0.425"

note "wrist_1_joint (eje Y, pose con 0.39225 en Z)"
check_joint "wrist_1_joint"       "0 1 0" "0 0 0.39225"

note "wrist_2_joint (eje Z, pose con 0.093 en Y)"
check_joint "wrist_2_joint"       "0 0 1" "0 0.093 0"

note "wrist_3_joint (eje Y, pose con 0.09465 en Z)"
check_joint "wrist_3_joint"       "0 1 0" "0 0 0.09465"

# Anti-patron explicito: detectar la migracion DH/axis-Z rota
note "anti-patron: poses DH (-0.425 / -0.39225) en joints del brazo"
if grep -nE '<pose relative_to="(upper_arm_link|forearm_link)">-0\.(425|39225)' "${SDF_PATH}" >/dev/null; then
    bad "detectada pose DH '-0.425' o '-0.39225' (convenio URDF, rompe meshes SDF)"
else
    ok  "no hay poses DH '-0.425 0 0' / '-0.39225 ...'"
fi

note "anti-patron: rotacion roll/pitch pi/2 inline en joints moviles del brazo"
# Solo los joints moviles. wrist_3_link queda fuera porque el unico pose con
# relative_to="wrist_3_link" es el EE fijo, que SI tiene -pi/2 legitimamente.
if grep -nE '<pose relative_to="(shoulder_link|wrist_1_link|wrist_2_link)">[^<]*1\.57079' "${SDF_PATH}" >/dev/null; then
    bad "detectada rotacion 1.57079 (pi/2) inline en joint del brazo (convenio URDF, rompe meshes)"
else
    ok  "no hay rotaciones 1.57079 inline en joints moviles del brazo"
fi

note "end_effector_frame_fixed_joint: pose debe ser '0 0 0 -1.57079... 0 0'"
ee_pose="$(awk '
    /<joint name="end_effector_frame_fixed_joint"/ { inj=1; next }
    inj && /<pose / { line=$0; sub(/^[[:space:]]+/,"",line); print line; exit }
' "${SDF_PATH}")"
if printf '%s' "${ee_pose}" | grep -qE '0 0 0 -1\.57079[0-9]* 0 0'; then
    ok  "end_effector_frame_fixed_joint pose correcto (${ee_pose})"
elif printf '%s' "${ee_pose}" | grep -qE '0 0 0 0 0 0</pose>'; then
    bad "end_effector_frame_fixed_joint pose es identidad (convenio URDF roto). actual: ${ee_pose}"
else
    bad "end_effector_frame_fixed_joint pose inesperado: ${ee_pose}"
fi

echo "---"
printf 'PASS=%d  FAIL=%d\n' "${PASS}" "${FAIL}"
if [ "${FAIL}" -eq 0 ]; then
    echo "GAZEBO_SDF_CONVENTION_OK=true"
    exit 0
else
    echo "GAZEBO_SDF_CONVENTION_OK=false"
    echo ""
    echo "Hint: si fallas en axis-Z masivo o poses DH, es la migracion rota."
    echo "  - inspeccionar:  git diff HEAD -- ${SDF_PATH}"
    echo "  - rescatar HEAD: git restore ${SDF_PATH}"
    exit 1
fi
