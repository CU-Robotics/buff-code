#! /bin/bash

#
# 	First install system dependencies through apt
#

echo -e "\n\tInstalling BuffCode Dependencies...\n"

$SUDO xargs apt install -y --no-install-recommends <${PROJECT_ROOT}/buffpy/data/install/dependencies.txt

#
#	Install pip with get-pip
# 		the system pip (from apt) is custom and
#		we would rather use vanilla pip (basic).

echo -e "\n\tInstalling pip3\n"

curl https://bootstrap.pypa.io/get-pip.py -o get-pip.py

$SUDO python3 get-pip.py

rm get-pip.py

#
#	Install python requirements with pip3
#

echo -e "\n\tInstalling BuffCode python3 requirements\n"

pip3 install -r ${PROJECT_ROOT}/buffpy/data/install/python3_requirements.txt
