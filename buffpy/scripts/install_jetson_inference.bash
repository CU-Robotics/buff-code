#! /bin/bash

# switch to the home directory
cd $HOME

# Clone the jetson inference repo
git clone --recursive https://github.com/dusty-nv/jetson-inference

# Switch to the jetson inference repo
cd jetson-inference

# Make the build dir and switch into it
mkdir build
cd build

# Build it using CMake
cmake ../
make -j$(nproc)

# Install the repo
sudo make install

# Setup links
sudo ldconfig