#!/bin/bash
# micro-ROS Agent 실행 스크립트
# Teensy 4.1 ↔ ROS2 Jazzy 브릿지 (USB Serial)
#
# 사용법:
#   bash scripts/start_microros_agent.sh [device] [baudrate]
#
# 기본값:
#   device   = /dev/ttyACM0
#   baudrate = 115200

DEVICE=${1:-/dev/ttyACM0}
BAUD=${2:-921600}   # match firmware default_transport.cpp Serial.begin(921600)

echo "[micro-ROS Agent] 디바이스: $DEVICE  전송속도: $BAUD"

# Docker 방식 (권장 — 별도 빌드 불필요)
if command -v docker &> /dev/null; then
    echo "[micro-ROS Agent] Docker로 실행 중..."
    docker run -it --rm \
        --net=host \
        -v /dev:/dev \
        --privileged \
        microros/micro-ros-agent:jazzy \
        serial --dev "$DEVICE" -b "$BAUD" -v4
else
    # native 방식 (micro_ros_agent 빌드 필요)
    echo "[micro-ROS Agent] native로 실행 중..."
    source /opt/ros/jazzy/setup.bash
    ros2 run micro_ros_agent micro_ros_agent serial --dev "$DEVICE" -b "$BAUD" -v4
fi
