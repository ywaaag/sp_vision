#!/bin/bash

# 进入脚本所在目录
cd /home/rm/Desktop/sp_vision_25 || exit 1

# 进程崩溃重启上限
MAX_RETRY=100
RETRY_COUNT=0

# 错误日志触发重启上限
MAX_ERROR_RETRY=50
ERROR_COUNT=0

# 定义退出函数和信号捕获
cleanup() {
  echo "检测到终止信号或错误上限，退出脚本..."
  pkill -P $$  # 杀死所有子进程
  exit 1
}
trap cleanup SIGINT SIGTERM

while true; do
  # 检查 sentry_multithread 进程是否存在
  if ! pidof sentry_multithread > /dev/null; then
    echo "sentry_multithread 未运行，正在重启..."

    RETRY_COUNT=$((RETRY_COUNT + 1))
    if [ "$RETRY_COUNT" -gt "$MAX_RETRY" ]; then
      echo "错误: 进程崩溃重启已达上限 ($MAX_RETRY 次)"
      cleanup
    fi

    sleep 1

    # 载入 ROS 环境
    source /opt/ros/humble/setup.bash
    source ~/Desktop/sp_msg_25/install/setup.bash
    export LD_LIBRARY_PATH=/opt/ros/humble/lib:$HOME/Desktop/sp_msg_25/install/sp_msgs/lib:$LD_LIBRARY_PATH

    # 运行程序并监控日志
    {
      ./build/sentry_multithread 2>&1 | while IFS= read -r line; do
        echo "$line"
        # 检测到 error 日志时触发重启
        if [[ "$line" == *"[error]"* ]]; then
          ERROR_COUNT=$((ERROR_COUNT + 1))
          echo "[监控] 检测到错误日志，错误重启计数: $ERROR_COUNT/$MAX_ERROR_RETRY"
          if [ "$ERROR_COUNT" -ge "$MAX_ERROR_RETRY" ]; then
            echo "错误: 错误重启已达上限 ($MAX_ERROR_RETRY 次)"
            cleanup
          fi
          echo "[监控] 3秒后重启程序..."
          sleep 3
          pkill sentry_multithread  # 杀死当前进程，触发主循环重启
        fi
      done
    } &

    # 等待程序结束
    wait

  else
    # 进程运行正常时重置崩溃计数器
    RETRY_COUNT=0
  fi

  sleep 5
done