"""Helpers internos extraidos de panel_pick_demo.py (refactor F3 Bloque B).

Modulo creado para descomponer panel_pick_demo.py (12.249 lineas) en
unidades mas manejables. Contiene las funciones modulo-nivel previas
a ``run_pick_demo`` (resolucion de objeto, validacion de transporte
y waits de progreso) que tienen API estable y dependencias acotadas.

``run_pick_demo`` (~11.000 L con closures anidadas) sigue en el archivo
principal y se atacara en pasos posteriores.
"""
