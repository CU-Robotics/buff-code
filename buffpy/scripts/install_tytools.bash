#! /bin/bash

#
#	Setup Teensy Rules
#

echo -e "\n\tInstalling Teensy Rules...\n"

curl https://www.pjrc.com/teensy/00-teensy.rules -O

# mv rules into rules.d and set the proper file permissions
$SUDO mv 00-teensy.rules /etc/udev/rules.d/00-teensy.rules
$SUDO chmod 0644 /etc/udev/rules.d/00-teensy.rules

# Recommended
curl -fsSL https://raw.githubusercontent.com/platformio/platformio-core/master/scripts/99-platformio-udev.rules | sudo tee /etc/udev/rules.d/99-platformio-udev.rules

# OR, manually download and copy this file to destination folder
sudo cp 99-platformio-udev.rules /etc/udev/rules.d/99-platformio-udev.rules

echo -e "\n\tInstalling TYcmd...\n"

cd "${PROJECT_ROOT}/.."

$SUDO apt -y install build-essential cmake libudev-dev qtbase5-dev pkg-config

wget https://github.com/Koromix/tytools/archive/refs/tags/v0.9.8.tar.gz

tar -xf v0.9.8.tar.gz

cd tytools-0.9.8

mkdir -p build/linux && cd build/linux
cmake -DCMAKE_INSTALL_PREFIX=/usr/local ../..
make

$SUDO make install

cd "${PROJECT_ROOT}"

rm -rf ../tytools-0.9.8