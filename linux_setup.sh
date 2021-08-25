#/bin/sh
# SETUP FOR PERSPECTIVE
# This is a complete end-to-end script to install all packages
# and tools needed to run the Perspective demo

set -e # exit if any command should error

sudo apt update

echo "===== Installing Bazelisk ====="
sudo apt install npm
sudo npm install -g @bazel/bazelisk

echo "===== Installing OpenCV/FFmpeg  ====="
sudo apt install libopencv-core-dev libopencv-highgui-dev \
                       libopencv-calib3d-dev libopencv-features2d-dev \
                       libopencv-imgproc-dev libopencv-video-dev

echo "===== Installing Mesa GPU libraries  ====="
sudo apt install mesa-common-dev libegl1-mesa-dev libgles2-mesa-dev

echo "===== Installing gcc8 ====="
sudo apt install gcc-8 g++-8
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-8 800 --slave /usr/bin/g++ g++ /usr/bin/g++-8

echo "===== Installing useful Unix tools (caffeine) ====="
sudo apt install gnome-shell-extension-caffeine

echo ""
echo ""
echo "Now modify WORKSPACE to say '/usr' instead of '/usr/lib' under opencv_linux"
