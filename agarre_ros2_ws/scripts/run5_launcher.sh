#!/bin/bash
pkill -f "gz-transport-topic" 2>/dev/null || true
pkill -f "gz sim" 2>/dev/null || true
sleep 2
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null || true
cd /home/laboratorio/TFM/agarre_ros2_ws
exec bash scripts/run_directo_validation.sh
