# Ruta/archivo: agarre_ros2_ws/Makefile
# Contenido: Atajos de build y operacion para el workspace ROS 2.
# Uso breve: Se ejecuta con make desde agarre_ros2_ws cuando aplica.
SHELL := /bin/bash

ROOT := $(abspath $(dir $(lastword $(MAKEFILE_LIST))))
VENV ?= /home/laboratorio/TFM/.venv-tfm
RUFF := $(if $(wildcard $(VENV)/bin/ruff),$(VENV)/bin/ruff,ruff)
PYTEST := $(if $(wildcard $(VENV)/bin/pytest),$(VENV)/bin/pytest,pytest)

.PHONY: qa lint format-check format test evidence-startup evidence-latest evidence-prune

qa: lint test

lint:
	@echo "[MAKE] lint"
	@$(RUFF) check src

format-check:
	@echo "[MAKE] format-check"
	@$(RUFF) format --check src

format:
	@echo "[MAKE] format"
	@$(RUFF) format src

test:
	@echo "[MAKE] test"
	@PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 $(PYTEST) --import-mode=importlib -q src/ur5_qt_panel/test src/ur5_tools/test

evidence-startup:
	@echo "[MAKE] evidence-startup"
	@cd "$(ROOT)" && ./scripts/evidence_startup_ready.sh $${CYCLES:-3}

evidence-latest:
	@echo "[MAKE] evidence-latest"
	@cd "$(ROOT)" && ./scripts/show_latest_startup_evidence.sh

evidence-prune:
	@echo "[MAKE] evidence-prune"
	@cd "$(ROOT)" && ./scripts/prune_startup_evidence.sh $${KEEP:-10}
