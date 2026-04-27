"""Mixins extraídos de ur5_moveit_bridge.py (refactor F3 plan-2026-04-27).

Cada submódulo aporta un mixin que se compone con la clase principal
``UR5MoveItBridge`` para mantener el archivo monolítico por debajo de 1.500 L
sin alterar las llamadas internas (``self._x`` siguen funcionando).
"""
