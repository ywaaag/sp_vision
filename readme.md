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
3. Activate GPU(Intel NUC only)
    ```
    mkdir neo  
    cd neo  

    wget https://github.com/intel/intel-graphics-compiler/releases/download/igc-1.0.13463.18/intel-igc-core_1.0.13463.18_amd64.deb  
    wget https://github.com/intel/intel-graphics-compiler/releases/download/igc-1.0.13463.18/intel-igc-opencl_1.0.13463.18_amd64.deb  
    wget https://github.com/intel/compute-runtime/releases/download/23.09.25812.14/intel-level-zero-gpu-dbgsym_1.3.25812.14_amd64.ddeb  
    wget https://github.com/intel/compute-runtime/releases/download/23.09.25812.14/intel-level-zero-gpu_1.3.25812.14_amd64.deb  
    wget https://github.com/intel/compute-runtime/releases/download/23.09.25812.14/intel-opencl-icd-dbgsym_23.09.25812.14_amd64.ddeb  
    wget https://github.com/intel/compute-runtime/releases/download/23.09.25812.14/intel-opencl-icd_23.09.25812.14_amd64.deb  
    wget https://github.com/intel/compute-runtime/releases/download/23.09.25812.14/libigdgmm12_22.3.0_amd64.deb  
    wget https://github.com/intel/compute-runtime/releases/download/23.09.25812.14/ww09.sum  

    sha256sum -c ww09.sum  
    sudo dpkg -i *.deb  
    ```

    Note: If use GPU **async-infer** the highest display resolution is **1920*1080(24Hz)**

4. Set up Serial Port
    1. Grant permission
        ```
        sudo usermod -a -G dialout $USER
        ```
    2. Get port ID(serial, idVendor and idProduct)
        ```
        udevadm info -a -n /dev/ttyACM0 | grep -E '({serial}|{idVendor}|{idProduct})'
        ```
        Replace /dev/ttyACM0 with your actual device name.
    3. Create udev rules file
        ```
        sudo touch /etc/udev/rules.d/99-usb-serial.rules
        ```
        Then put the following into the file(Replace content with real ID, SYMLINK is the serial port name after fix):
        ```
        SUBSYSTEM=="tty", ATTRS{idVendor}=="1234", ATTRS{idProduct}=="1234", ATTRS{serial}=="A1234567", SYMLINK+="gimbal"

        ```
    4. Reload udev rules
        ```
        sudo udevadm control --reload-rules
        sudo udevadm trigger
        ```
    5. Check
        ```
        ls -l /dev/gimbal
        # Expected output (example):
        # lrwxrwxrwx 1 root root 7 Jul 21 10:00 /dev/gimbal -> ttyACM0
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
        openssh-server \
        screen
    ```
2. Install [OpenVINO](https://docs.openvino.ai/2024/get-started/install-openvino/install-openvino-archive-linux.html)
3. Install [Ceres](http://ceres-solver.org/installation.html)
4. Build:
    ```bash
    cmake -B build
    make -C build/ -j`nproc`
    ```
5. Verify:
    ```bash
    ./build/auto_aim_test
    ```
6. Autostart:
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
        Exec=/home/rm/Desktop/sp_vision_25/autostart.sh
        Name=sp_vision
        ```
        Note: [Exec](https://specifications.freedesktop.org/desktop-entry-spec/desktop-entry-spec-latest.html) must be absolute path.
    4. Make sure `autostart.sh` has the permission to execute:
        ```
        chmod +x autostart.sh
        ```