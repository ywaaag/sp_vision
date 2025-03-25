#!/bin/bash

# 进入脚本所在目录
cd /home/rm/Desktop/sp_vision_25 || exit 1

# 进程崩溃重启上限
MAX_RETRY=100
RETRY_COUNT=0

# 定义退出函数和信号捕获
cleanup() {
  echo "检测到终止信号，退出脚本..."
  pkill -P $$  # 杀死所有子进程
  exit 1
}
trap cleanup SIGINT SIGTERM

while true; do
  # 检查 standard 进程是否存在
  if ! pidof standard > /dev/null; then
    echo "standard 未运行，正在重启..."

    RETRY_COUNT=$((RETRY_COUNT + 1))
    if [ "$RETRY_COUNT" -gt "$MAX_RETRY" ]; then
      echo "错误: 进程崩溃重启已达上限 ($MAX_RETRY 次)"
      cleanup
    fi

    sleep 1

    # 启动程序（后台运行）
    ./build/standard ./configs/swerve.yaml 2>&1 &

  else
    # 进程运行正常时重置崩溃计数器
    RETRY_COUNT=0
  fi

  sleep 5
done