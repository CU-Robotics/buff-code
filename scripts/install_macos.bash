#! /usr/bin/env bash

#  run this script with 'source install.bash'

DIR=$PWD

echo -e "Running Install from ${PWD}"
echo -e "\n\tbrew updating...\n"

brew update
brew upgrade

echo -e "\n\tInstalling Dependencies...\n"

<<<<<<< HEAD
<<<<<<< HEAD
xargs brew install <${PWD}/config/install/dependencies.txt 
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | BINDIR=${DIR}/buffpy/bin sh

pip3 install -r ${PWD}/config/install/python3_requirements.txt
=======
=======
>>>>>>> 47d965f662e63992007a5497e11b859e9e42ef2c
xargs brew install <${PROJECT_ROOT}/config/install/dependencies.txt 
brew install arduino-cli

pip3 install -r ${PROJECT_ROOT}/config/install/python3_requirements.txt
<<<<<<< HEAD
>>>>>>> cf627da52f077c0b7d2535e30334532294b8415c
=======
>>>>>>> 47d965f662e63992007a5497e11b859e9e42ef2c

export PATH="${PROJECT_ROOT}/buffpy/bin:${PATH}"
export PYTHONPATH="${PROJECT_ROOT}/buffpy/lib:${PYTHONPATH}"
