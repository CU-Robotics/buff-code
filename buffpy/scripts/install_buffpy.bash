#! /bin/bash


#
# 	First install system dependencies through apt
#

echo -e "\n\tInstalling BuffCode Dependencies...\n"

sudo xargs apt install -y --no-install-recommends <${PROJECT_ROOT}/buffpy/config/install/dependencies.txt


#
#	Install pip with get-pip
#

echo -e "\n\tInstalling pip3\n"

curl https://bootstrap.pypa.io/pip/3.6/get-pip.py -o get-pip.py

sudo python3 get-pip.py

rm get-pip.py


#
#	Install python requirements with pip3
#

echo -e "\n\tInstalling BuffCode python3 requirements\n"

pip install -r ${PROJECT_ROOT}/buffpy/config/install/python3_requirements.txt