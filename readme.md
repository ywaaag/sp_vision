## Deploy

### Prerequisites
1. Install [MindVision SDK](https://mindvision.com.cn/category/software/sdk-installation-package/) and [HikRobot SDK](https://www.hikrobotics.com/cn2/source/support/software/MVS_STD_GML_V2.1.2_231116.zip)
2. Setup USB2CAN
    1. Create rules file:
        ```
        sudo touch /etc/udev/rules.d/99-can-up.rules
        ```
    2. Put the following into the file:
        ```
        ACTION=="add", KERNEL=="can0", RUN+="/sbin/ip link set can0 up type can bitrate 1000000"
        ACTION=="add", KERNEL=="can1", RUN+="/sbin/ip link set can1 up type can bitrate 1000000"
        ```

### Ubuntu 22.04
1. Install other dependencies:
    ```bash
    sudo apt install -y \
        git \
        g++ \
        cmake \
        can-utils \
        libopencv-dev \
        libfmt-dev \
        libeigen3-dev \
        libspdlog-dev \
        libyaml-cpp-dev \
        libusb-1.0-0-dev \
        nlohmann-json3-dev \
        screen
    ```
2. Build:
    ```bash
    cmake -B build
    make -C build/ -j`nproc`
    ```
3. Verify:
    ```bash
    ./build/auto_aim_test
    ```
4. Autostart:
    1. Make sure `screen` has been installed:
        ```
        sudo apt install screen
        ```
    2. Create `.desktop` file:
        ```
        mkdir ~/.config/autostart/
        touch ~/.config/autostart/sp_vision.desktop
        ```
    3. Put the following into the file:
        ```
        [Desktop Entry]
        Type=Application
        Exec=/home/rm/Desktop/sp_vision/autostart.sh
        Name=sp_vision
        ```
        Note: [Exec](https://specifications.freedesktop.org/desktop-entry-spec/desktop-entry-spec-latest.html) must be absolute path.
    4. Make sure `autostart.sh` has the permission to execute:
        ```
        chmod +x autostart.sh
        ```

### Ubuntu 20.04
1. Install [Docker](https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository) with [non-root user setting](https://docs.docker.com/engine/install/linux-postinstall/#manage-docker-as-a-non-root-user)
2. Create docker image:
    ```bash
    docker build -t sp_vision .
    # to enable gui:
    xhost +
    ```
3. Build:
    ```bash
    ./docker_run.sh
    # inside the container:
    cmake -B build
    make -C build/ -j`nproc`
    ```
4. Verify:
    ```bash
    ./docker_run.sh
    # inside the container:
    ./build/auto_aim_test
    ```
