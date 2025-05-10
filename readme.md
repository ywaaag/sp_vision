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

4. Set up Serial Port(**Minimum-Vision-System only**)
    1. Create rules file:
        ```
        sudo touch /etc/udev/rules.d/99-ttyacm.rules
        ```
    2. Put the following into the file:
        ```
        SUBSYSTEM=="tty", ATTRS{idVendor}=="ffff", ATTRS{idProduct}=="ffff", MODE="0666"
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
2. Install [OpenVINO](https://docs.openvino.ai/2023.3/openvino_docs_install_guides_installing_openvino_from_archive_linux.html)
3. Build:
    ```bash
    cmake -B build
    make -C build/ -j`nproc`
    ```
4. Verify:
    ```bash
    ./build/auto_aim_test
    ```
5. Autostart:
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