#!/bin/bash

# 进入脚本所在目录（假设原脚本在 sp_vision_25 目录下）
cd ~/Desktop/sp_vision_25/

while true; do
  # 检查 sentry_multithread 进程是否存在
  if ! pgrep -x "sentry_multithread" > /dev/null; then
    echo "sentry_multithread 未运行，正在重启..."

    # 如果原脚本没有独立文件，直接将命令写在这里更可靠
    sleep 5
    source /opt/ros/humble/setup.bash
    source ~/Desktop/sp_msg_25/install/setup.bash
    export LD_LIBRARY_PATH=/opt/ros/humble/lib:~/Desktop/sp_msg_25/install/sp_msgs/lib:$LD_LIBRARY_PATH
    gnome-terminal -- bash -c "./build/sentry_multithread; exec bash"
  fi
  # 每隔 5 秒检查一次
  sleep 5
done