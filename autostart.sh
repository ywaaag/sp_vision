sleep 2
cd ~/Desktop/sp_vision_25/
gnome-terminal -x bash -c "./watchdog.sh;exec bash"
# screen \
#     -L \
#     -Logfile logs/$(date "+%Y-%m-%d_%H-%M-%S").screenlog \
#     -d \
#     -m \
#     bash -c "export LD_LIBRARY_PATH=/opt/ros/humble/lib:/home/rm/Desktop/sp_msg_25/install/lib:$LD_LIBRARY_PATH; ./build/sentry_multithread configs/sentry.yaml"
