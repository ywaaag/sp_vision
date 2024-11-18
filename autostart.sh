sleep 1
cd ~/Desktop/sp_vision/
screen \
    -L \
    -Logfile logs/$(date "+%Y-%m-%d_%H-%M-%S").screenlog \
    -d \
    -m \
    bash -c "./build/sentry configs/sentry.yaml"
