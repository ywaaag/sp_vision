#!/bin/bash

# 进入脚本所在目录
cd /home/rm/Desktop/sp_vision_25 || exit 1

MAX_RETRY=100
RETRY_COUNT=0

while true; do
  # 检查 sentry_multithread 进程是否存在
  if ! pidof sentry_multithread > /dev/null; then
    echo "sentry_multithread 未运行，正在重启..."

    RETRY_COUNT=$((RETRY_COUNT + 1))
    if [ "$RETRY_COUNT" -gt "$MAX_RETRY" ]; then
      echo "错误: 进程已多次崩溃，停止自动重启"
      exit 1
    fi

    sleep 1

    # 载入 ROS 环境
    source /opt/ros/humble/setup.bash
    source ~/Desktop/sp_msg_25/install/setup.bash
    export LD_LIBRARY_PATH=/opt/ros/humble/lib:$HOME/Desktop/sp_msg_25/install/sp_msgs/lib:$LD_LIBRARY_PATH

    # 直接运行 sentry_multithread（前台运行）
    ./build/sentry_multithread
  else
    RETRY_COUNT=0  # 进程运行正常，重置失败计数
  fi

  sleep 5
done
