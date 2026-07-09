#!/usr/bin/env bash
set -e

# 默认串口按实测环境填写。设备名不同的话，运行前改这里或设置环境变量。
ASR_PORT="${ASR_PORT:-/dev/ttyS10}"
FACE_PORT="${FACE_UART_DEVICE:-/dev/ttyACM0}"

# 调试阶段使用 chmod 最直接；正式部署建议使用 scripts/99-study-robot.rules。
sudo chmod 666 "$ASR_PORT" 2>/dev/null || true
sudo chmod 666 "$FACE_PORT" 2>/dev/null || true

FACE_UART_DEVICE="$FACE_PORT" ./pose_detector
