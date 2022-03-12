#! /bin/bash

echo -e "\n\tInstalling BuffCode Dependencies...\n"
#	Using apt and pip install all the dependencies for the project
# sudo 
xargs apt install -y <${PROJECT_ROOT}/buffpy/config/install/dependencies.txt
# sudo
rm -rf /var/lib/apt/lists/*

echo -e "\n\tUpgrading pip3\n"
# Not sure whats happening apt only installs pip==9.0.1
# sudo
apt purge -y python3-pip
# upgrade pip before installing dependencies
python3 -m pip install --upgrade pip==21.3.1

echo -e "\n\tInstalling BuffCode python3 requirements\n"
python3 -m pip install -r ${PROJECT_ROOT}/buffpy/config/install/python3_requirements.txt