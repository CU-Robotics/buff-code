sudo apt update && sudo apt upgrade -y && sudo apt dist-upgrade -y

RUN cd ${HOME} && \
	git clone https://github.com/IntelRealSense/librealsense.git && \
	cd librealsense && mkdir -p build && cd build && \
	cmake ../ -DBUILD_PYTHON_BINDINGS:bool=true -DFORCE_RSUSB_BACKEND:bool=true -DBUILD_WITH_CUDA:bool=false -DBUILD_GRAPHICAL_EXAMPLES:bool=false -DCMAKE_BUILD_TYPE=release && \
	sudo make uninstall && make clean && make && sudo make install