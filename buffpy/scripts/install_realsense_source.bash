sudo apt update && sudo apt upgrade -y && sudo apt dist-upgrade -y

cd ${HOME} && rm -rf librealsense && \
	git checkout https://github.com/IntelRealSense/librealsense.git && git checkout 23b0904 && cd librealsense && mkdir -p build && cd build && \
	cmake ../ -DBUILD_PYTHON_BINDINGS:bool=true -DFORCE_RSUSB_BACKEND:bool=true -DBUILD_WITH_CUDA:bool=false -DBUILD_GRAPHICAL_EXAMPLES:bool=true -DCMAKE_BUILD_TYPE=debug -DPYTHON_EXECUTABLE=`which python3.8` && \
	sudo make uninstall && make clean && make -j4 && sudo make install
	
echo 'export PYTHONPATH="$PYTHONPATH:/usr/lib/python3.8/site-packages/pyrealsense2"' >> ~/.bashrc
