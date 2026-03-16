#!/usr/bin/env bash
# check_qt_env.sh
# Script para comprobar entorno gráfico y Qt antes de lanzar el panel

set -e

# Comprobar DISPLAY
if [[ -z "$DISPLAY" ]]; then
  echo "[ERROR] No está definida la variable DISPLAY. Abre una terminal gráfica (no TTY/SSH) y vuelve a intentarlo."
  exit 1
else
  echo "[OK] DISPLAY=$DISPLAY"
fi

# Comprobar XAUTHORITY
if [[ -z "$XAUTHORITY" ]]; then
  if [[ -f "/run/user/$(id -u)/gdm/Xauthority" ]]; then
    export XAUTHORITY="/run/user/$(id -u)/gdm/Xauthority"
    echo "[INFO] XAUTHORITY no estaba definido, usando $XAUTHORITY"
  else
    echo "[ERROR] No está definida la variable XAUTHORITY y no se encuentra archivo de autenticación X."
    exit 2
  fi
else
  echo "[OK] XAUTHORITY=$XAUTHORITY"
fi

# Comprobar plugins de Qt
QT_PLUGIN_PATH_OK=0
QT_QPA_PLATFORM_PLUGIN_PATH_OK=0
if [[ -d "/usr/lib/x86_64-linux-gnu/qt5/plugins/platforms" ]]; then
  QT_PLUGIN_PATH_OK=1
  QT_QPA_PLATFORM_PLUGIN_PATH_OK=1
fi
if [[ $QT_PLUGIN_PATH_OK -eq 1 ]]; then
  export QT_PLUGIN_PATH="/usr/lib/x86_64-linux-gnu/qt5/plugins"
  echo "[OK] QT_PLUGIN_PATH configurado."
fi
if [[ $QT_QPA_PLATFORM_PLUGIN_PATH_OK -eq 1 ]]; then
  export QT_QPA_PLATFORM_PLUGIN_PATH="/usr/lib/x86_64-linux-gnu/qt5/plugins/platforms"
  echo "[OK] QT_QPA_PLATFORM_PLUGIN_PATH configurado."
fi

# Comprobar si el plugin xcb existe
if [[ ! -f "/usr/lib/x86_64-linux-gnu/qt5/plugins/platforms/libqxcb.so" ]]; then
  echo "[ERROR] No se encuentra el plugin libqxcb.so. Instala el paquete: sudo apt-get install qt5-qtbase qt5-qmake qtbase5-dev qtbase5-dev-tools libxcb-xinerama0"
  exit 3
else
  echo "[OK] Plugin libqxcb.so encontrado."
fi

# Comprobar dependencias del plugin xcb
if ! ldd /usr/lib/x86_64-linux-gnu/qt5/plugins/platforms/libqxcb.so | grep 'not found'; then
  echo "[OK] Todas las dependencias de libqxcb.so están presentes."
else
  echo "[ERROR] Faltan dependencias para libqxcb.so. Instala los paquetes necesarios."
  exit 4
fi

echo "[OK] Entorno gráfico y Qt listos para lanzar el panel."
exit 0
