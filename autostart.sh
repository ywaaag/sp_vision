sleep 1
cd ~/Desktop/sp_vision_25/
screen \
    -L \
    -Logfile logs/$(date "+%Y-%m-%d_%H-%M-%S").screenlog \
    -d \
    -m \
<<<<<<< HEAD
    bash -c "./build/standard configs/standard3.yaml"
=======
    bash -c "./build/sentry configs/sentry.yaml"
>>>>>>> auto_aim_sentry
