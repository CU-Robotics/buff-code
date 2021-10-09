#! /usr/bin/env bash

#  run this script with 'source install.bash'

DIR=$PWD

echo -e "Running Install from ${PWD}"
echo -e "\n\tbrew updating...\n"

brew update
brew upgrade

echo -e "\n\tInstalling Dependencies...\n"

<<<<<<< HEAD
xargs brew install <${PWD}/config/install/dependencies.txt 
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | BINDIR=${DIR}/buffpy/bin sh

pip3 install -r ${PWD}/config/install/python3_requirements.txt
=======
xargs brew install <${PROJECT_ROOT}/config/install/dependencies.txt 
brew install arduino-cli

pip3 install -r ${PROJECT_ROOT}/config/install/python3_requirements.txt
>>>>>>> cf627da52f077c0b7d2535e30334532294b8415c

export PATH="${PROJECT_ROOT}/buffpy/bin:${PATH}"
export PYTHONPATH="${PROJECT_ROOT}/buffpy/lib:${PYTHONPATH}"
