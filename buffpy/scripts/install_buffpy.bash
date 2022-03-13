#! /bin/bash


#
# 	First install system dependencies through apt
#
echo -e "\n\tInstalling BuffCode Dependencies...\n"

sudo xargs apt install -y <${PROJECT_ROOT}/buffpy/config/install/dependencies.txt


#
#	Upgrade pip to recent version (21.3.1)
#
echo -e "\n\tUpgrading pip3\n"

python3 -m pip install --upgrade pip==21.3.1 # upgrade pip3

sudo apt purge -y python3-pip # Remove distro pip3 # Apt installs and can't uninstall pip3==9.0.1


#
#	Install python requirements with pip3
#
echo -e "\n\tInstalling BuffCode python3 requirements\n"

python3 -m pip install -r ${PROJECT_ROOT}/buffpy/config/install/python3_requirements.txt