# Ubuntu/Debian steps from https://wiki.ros.org/noetic/Installation/Source
# 1. Prerequisites
sudo apt-get install python3-rosdep python3-rosinstall-generator python3-vcstools python3-vcstool build-essential

# 1.2 initialize rosdep
sudo rosdep init
rosdep update


# 2. Installation
# 2.1 Create a catkin Workspace
mkdir ~/ros_catkin_ws
cd ~/ros_catkin_ws

# download ROS source packages
rosinstall_generator desktop --rosdistro noetic --deps --tar > noetic-desktop.rosinstall
mkdir ./src
vcs import --input noetic-desktop.rosinstall ./src

# 2.1.1 Resolving Dependencies
rosdep install --from-paths ./src --ignore-packages-from-source --rosdistro noetic -y

# 2.1.2 Building the catkin Workspace
## ensure this uses python3. Append to following command if python is broken, updating the path accordingly: -DPYTHON_EXECUTABLE=/usr/bin/python3
./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release\
source ~/ros_catkin_ws/install_isolated/setup.bash


# 3. Maintaining a Source Checkout
# 3.1 Update your rosinstall file
mv -i noetic-desktop.rosinstall noetic-desktop.rosinstall.old
rosinstall_generator desktop --rosdistro noetic --deps --tar > noetic-desktop.rosinstall
diff -u noetic-desktop.rosinstall noetic-desktop.rosinstall.old # this might not be required for automated install

# 3.2 Download the latest sources
vcs import --input noetic-desktop.rosinstall ./src

# 3.3 Rebuild your workspace
./src/catkin/bin/catkin_make_isolated --install
source ~/ros_catkin_ws/install_isolated/setup.bash