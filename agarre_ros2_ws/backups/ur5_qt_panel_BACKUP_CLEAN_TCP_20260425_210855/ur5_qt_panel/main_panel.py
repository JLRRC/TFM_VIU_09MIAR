#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/main_panel.py
# Contenido: Alias de nomenclatura TFM para panel_v2.py.
# Uso breve: Entry point alternativo; toda la logica reside en panel_v2.py.
# URL: /home/laboratorio/TFM/agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/main_panel.py
# Summary: Thin wrapper that re-exports panel_v2 under the TFM canonical name.
"""Alias de nomenclatura TFM para el panel de control Qt del UR5.

La implementacion operativa reside en panel_v2.py.  Este modulo existe
unicamente para alinear el entry point con la denominacion usada en la
memoria del TFM (``main_panel``), sin duplicar ni reescribir logica.

Uso::

    ros2 run ur5_qt_panel main_panel
    python -m ur5_qt_panel.main_panel
"""
from ur5_qt_panel.panel_v2 import ControlPanelV2, main  # noqa: F401

__all__ = ['ControlPanelV2', 'main']

if __name__ == '__main__':
    main()
