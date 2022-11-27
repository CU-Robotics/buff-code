#! /bin/bash


#
# 	First install system dependencies through apt
#

echo -e "\n\tInstalling BuffCode Dependencies...\n"

$SUDO xargs apt install -y --no-install-recommends <${PROJECT_ROOT}/buffpy/config/install/dependencies.txt


#
#	Install pip with get-pip
#

echo -e "\n\tInstalling pip3\n"

curl https://bootstrap.pypa.io/pip/3.6/get-pip.py -o get-pip.py

$SUDO python3 get-pip.py

rm get-pip.py


#
#	Install python requirements with pip3
#

echo -e "\n\tInstalling BuffCode python3 requirements\n"

pip3 install -r ${PROJECT_ROOT}/buffpy/config/install/python3_requirements.txt

# if [[ "${HOSTNAME}" == *"edge"* ]]; then
# 	wget https://nvidia.box.com/shared/static/h1z9sw4bb1ybi0rm3tu8qdj8hs05ljbm.whl -O torch-1.9.0-cp36-cp36m-linux_aarch64.whl
# 	$SUDO apt install -y --no-install-recommends libopenblas-base libopenmpi-dev
# 	pip3 install --force-reinstall torch-1.9.0-cp36-cp36m-linux_aarch64.whl
# fi
