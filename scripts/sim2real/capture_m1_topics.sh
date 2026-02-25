#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 ]]; then
  echo "Usage: $0 <run_dir> [joint_topic] [imu_topic] [clock_topic]"
  echo "Example: $0 logs/sim2real/20260225-120000 /rb/joint_states /rb/imu /clock"
  exit 1
fi

RUN_DIR="$1"
JOINT_TOPIC="${2:-/rb/joint_states}"
IMU_TOPIC="${3:-/rb/imu}"
CLOCK_TOPIC="${4:-/clock}"

mkdir -p "${RUN_DIR}"

TOPIC_LIST_OUT="${RUN_DIR}/topic_list.txt"
HZ_OUT="${RUN_DIR}/topic_hz_joint_states.txt"
IMU_ECHO_OUT="${RUN_DIR}/topic_echo_imu.txt"
CLOCK_ECHO_OUT="${RUN_DIR}/topic_echo_clock.txt"
JOINT_ECHO_OUT="${RUN_DIR}/topic_echo_joint_states_once.txt"

echo "[CAPTURE] run_dir         : ${RUN_DIR}"
echo "[CAPTURE] joint_topic     : ${JOINT_TOPIC}"
echo "[CAPTURE] imu_topic       : ${IMU_TOPIC}"
echo "[CAPTURE] clock_topic     : ${CLOCK_TOPIC}"

ros2 topic list | sort > "${TOPIC_LIST_OUT}"
timeout 5s ros2 topic hz "${JOINT_TOPIC}" > "${HZ_OUT}" 2>&1 || true
timeout 5s ros2 topic echo --once "${IMU_TOPIC}" > "${IMU_ECHO_OUT}" 2>&1 || true
timeout 5s ros2 topic echo --once "${CLOCK_TOPIC}" > "${CLOCK_ECHO_OUT}" 2>&1 || true
timeout 5s ros2 topic echo --once "${JOINT_TOPIC}" > "${JOINT_ECHO_OUT}" 2>&1 || true

echo "[CAPTURE] saved:"
echo "  - ${TOPIC_LIST_OUT}"
echo "  - ${HZ_OUT}"
echo "  - ${IMU_ECHO_OUT}"
echo "  - ${CLOCK_ECHO_OUT}"
echo "  - ${JOINT_ECHO_OUT}"
