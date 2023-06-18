sudo pip install pybind11
sudo pip install pyrealsense2

sudo apt update && sudo apt upgrade -y && sudo apt dist-upgrade -y

cd ${HOME} && \
	git clone https://github.com/IntelRealSense/librealsense.git && \
	cd librealsense && git checkout b874e42 && mkdir -p build && cd build && \
	cmake ../ -DBUILD_PYTHON_BINDINGS:bool=true -DFORCE_RSUSB_BACKEND:bool=true -DBUILD_WITH_CUDA:bool=false -DBUILD_GRAPHICAL_EXAMPLES:bool=false -DCMAKE_BUILD_TYPE=release -DPYTHON_EXECUTABLE=`which python3.8` && \
	sudo make uninstall && make clean && make -j4 && sudo make install
