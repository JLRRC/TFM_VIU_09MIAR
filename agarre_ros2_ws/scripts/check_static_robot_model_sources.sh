#!/usr/bin/env bash
# Ruta/archivo: agarre_ros2_ws/scripts/check_static_robot_model_sources.sh
# Resumen: chequeo estatico (SIN Gazebo vivo) que valida que las fuentes del
#          robot esten en buen estado antes de arrancar el stack:
#            - URDF/Xacro existe y se procesa.
#            - model.sdf existe y usa convenio SDF correcto.
#            - world incluye UN solo model://ur5_rg2.
#            - No hay ros_gz_sim create del robot en launches.
#            - No hay model.sdf alternativo inesperado fuera del paquete.
# Uso:    ./check_static_robot_model_sources.sh
# Salida: exit 0 si todo OK, exit 1 si algo falla.

set -u

WS_DIR="${WS_DIR:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
URDF_XACRO="${URDF_XACRO:-${WS_DIR}/src/ur5_description/urdf/ur5.urdf.xacro}"
MODEL_SDF="${MODEL_SDF:-${WS_DIR}/src/ur5_gazebo/models/ur5_rg2/model.sdf}"
WORLD_SDF="${WORLD_SDF:-${WS_DIR}/src/ur5_gazebo/worlds/ur5_mesa_objetos.sdf}"
TMP_URDF="${TMP_URDF:-/tmp/ur5_rg2.urdf}"

PASS=0
FAIL=0

note() { printf '[CHK]  %s\n' "$*"; }
ok()   { PASS=$((PASS + 1)); printf '[PASS] %s\n' "$*"; }
bad()  { FAIL=$((FAIL + 1)); printf '[FAIL] %s\n' "$*" >&2; }
warn() { printf '[WARN] %s\n' "$*"; }

# (1) URDF/Xacro existe
note "URDF/Xacro presente"
if [ -f "${URDF_XACRO}" ]; then
    ok  "${URDF_XACRO}"
else
    bad "URDF/Xacro ausente: ${URDF_XACRO}"
fi

# (2) xacro genera URDF
note "xacro -> ${TMP_URDF}"
if command -v xacro >/dev/null 2>&1; then
    if xacro "${URDF_XACRO}" > "${TMP_URDF}" 2>/tmp/xacro_err_$$.log; then
        ok  "xacro genero URDF ($(wc -l < "${TMP_URDF}") lineas)"
    else
        bad "xacro fallo:"
        sed 's/^/      /' /tmp/xacro_err_$$.log >&2 || true
    fi
    rm -f /tmp/xacro_err_$$.log
else
    warn "xacro no esta en PATH (saltando generacion)"
fi

# (3) check_urdf opcional
note "check_urdf (si disponible)"
if command -v check_urdf >/dev/null 2>&1 && [ -f "${TMP_URDF}" ]; then
    if check_urdf "${TMP_URDF}" >/tmp/check_urdf_out_$$.log 2>&1; then
        ok  "check_urdf paso"
    else
        bad "check_urdf fallo:"
        sed 's/^/      /' /tmp/check_urdf_out_$$.log >&2 || true
    fi
    rm -f /tmp/check_urdf_out_$$.log
else
    warn "check_urdf no disponible (o URDF no generado)"
fi

# (4) model.sdf existe
note "model.sdf presente"
if [ -f "${MODEL_SDF}" ]; then
    ok  "${MODEL_SDF}"
else
    bad "model.sdf ausente: ${MODEL_SDF}"
fi

# (5) Convenio SDF correcto (no migracion DH rota)
note "convencion SDF (delegado a check_ur5_rg2_sdf_convention.sh)"
CONV_SCRIPT="${WS_DIR}/scripts/check_ur5_rg2_sdf_convention.sh"
if [ -x "${CONV_SCRIPT}" ]; then
    if "${CONV_SCRIPT}" >/tmp/conv_$$.log 2>&1; then
        ok  "check_ur5_rg2_sdf_convention paso"
    else
        bad "check_ur5_rg2_sdf_convention fallo. Detalle:"
        sed 's/^/      /' /tmp/conv_$$.log >&2 || true
    fi
    rm -f /tmp/conv_$$.log
else
    bad "script de convenio SDF ausente o no ejecutable: ${CONV_SCRIPT}"
fi

# (6) World con UN include model://ur5_rg2
note "world ${WORLD_SDF}: UN include model://ur5_rg2"
if [ -f "${WORLD_SDF}" ]; then
    INC_COUNT="$(grep -cE '<uri>[[:space:]]*model://ur5_rg2[[:space:]]*</uri>' "${WORLD_SDF}" || true)"
    if [ "${INC_COUNT}" -eq 1 ]; then
        ok  "world contiene exactamente 1 include de model://ur5_rg2"
    elif [ "${INC_COUNT}" -eq 0 ]; then
        bad "world NO contiene <include><uri>model://ur5_rg2</uri></include>"
    else
        bad "world contiene ${INC_COUNT} includes de model://ur5_rg2 (DUPLICADO)"
    fi
else
    bad "world ausente: ${WORLD_SDF}"
fi

# (7) Ningun ros_gz_sim create del robot en launches
note "ros_gz_sim create del robot en launches"
SUSPECT_CREATE="$(grep -rnE "ros_gz_sim.*create|gz[[:space:]]+service.*create" "${WS_DIR}/src" --include="*.py" 2>/dev/null | grep -vE "(test_|scripts/|panel_object_mgmt|spawn_objects|release_objects|panel_gz_objects)" | grep -iE "robot_description|ur5_rg2|-topic|-name|spawn" || true)"
if [ -z "${SUSPECT_CREATE}" ]; then
    ok  "no hay ros_gz_sim create del robot en launches"
else
    bad "posibles spawns del robot detectados:"
    printf '%s\n' "${SUSPECT_CREATE}" | sed 's/^/      /' >&2
fi

# (8) Solo un model.sdf de ur5_rg2 en el repo
note "model.sdf de ur5_rg2 en repo"
SDF_HITS="$(find "${WS_DIR}/src" -type f -name "model.sdf" -path "*ur5_rg2*" 2>/dev/null)"
N="$(printf '%s\n' "${SDF_HITS}" | grep -c . || true)"
if [ "${N}" -eq 1 ]; then
    ok  "unico model.sdf de ur5_rg2: ${SDF_HITS}"
elif [ "${N}" -eq 0 ]; then
    bad "no se encontro ningun model.sdf de ur5_rg2"
else
    bad "multiples model.sdf de ur5_rg2 (${N}):"
    printf '%s\n' "${SDF_HITS}" | sed 's/^/      /' >&2
fi

# (9) Modelos top-level alternativos no documentados
note "modelos alternativos del robot fuera de src/ur5_gazebo/models/ur5_rg2/"
OUTSIDE="$(find "${WS_DIR}" \
    -path "${WS_DIR}/build" -prune -o \
    -path "${WS_DIR}/install" -prune -o \
    -path "${WS_DIR}/log" -prune -o \
    -path "${WS_DIR}/.git" -prune -o \
    -type f -name "model.sdf" -print 2>/dev/null \
  | grep -vE "src/ur5_gazebo/models/ur5_rg2/model\.sdf$" || true)"
if [ -z "${OUTSIDE}" ]; then
    ok  "no hay model.sdf adicionales fuera del paquete oficial"
else
    warn "model.sdf adicionales detectados (revisar si son legitimos):"
    printf '%s\n' "${OUTSIDE}" | sed 's/^/      /'
fi

# (10) Resumen
echo "---"
printf 'PASS=%d  FAIL=%d\n' "${PASS}" "${FAIL}"
if [ "${FAIL}" -eq 0 ]; then
    echo "STATIC_ROBOT_MODEL_SOURCES_OK=true"
    exit 0
else
    echo "STATIC_ROBOT_MODEL_SOURCES_OK=false"
    exit 1
fi
