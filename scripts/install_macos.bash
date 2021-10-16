#! /usr/bin/env bash

#  run this script with 'source install.bash'

DIR=$PWD

echo -e "Running Install from ${PWD}"
echo -e "\n\tbrew updating...\n"

brew update
brew upgrade

#echo -e "\n\tInstalling Dependencies...\n"

#xargs brew install <${PWD}/config/install/dependencies.txt 
#curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | BINDIR=${DIR}/buffpy/bin sh

#pip3 install -r ${PWD}/config/install/python3_requirements.txt

#export PATH="${PROJECT_ROOT}/buffpy/bin:${PATH}"
#export PYTHONPATH="${PROJECT_ROOT}/buffpy/lib:${PYTHONPATH}"

curl https://www.pjrc.com/teensy/teensy_linux64.tar.gz -O
# teensy rules file
curl https://www.pjrc.com/teensy/00-teensy.rules -O

# mv rules into rules.d and set the proper file permissions
sudo mkdir -p /etc/udev/rules.d
sudo mv 00-teensy.rules /etc/udev/rules.d/
sudo chmod 0644 /etc/udev/rules.d/00-teensy.rules

# extract the tar to buffpy/bin
tar -xzvf teensy_linux64.tar.gz -C ${PROJECT_ROOT}/buffpy/bin
# remove unecessary tar.gz 
rm teensy_linux64.tar.gz