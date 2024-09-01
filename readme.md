## Some instructions
- This branch will provide the common code for `sp_vision`, and other branches need to **be created from this branch**
- Other branches should be named after algorithm functions, such as `auto_aim`,`auto_buff`,`auto_aim_sentry`

- Place the algorithm function code in the `tasks` folder and the algorithm test code in the `examples` folder.

- The src folder contains c++ files named after the robot, such as `standard3.cpp` `hero.cpp`

- **Merge** the algorithm with the branch when function is **mature and stable**
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
