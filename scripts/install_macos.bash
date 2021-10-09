#! /usr/bin/env bash

#  run this script with 'source install.bash'

DIR=$PWD

echo -e "Running Install from ${PWD}"
echo -e "\n\tbrew updating...\n"

brew update
brew upgrade

echo -e "\n\tInstalling Dependencies...\n"

xargs brew install <${PROJECT_ROOT}/config/install/dependencies.txt 
brew install arduino-cli

pip3 install -r ${PROJECT_ROOT}/config/install/python3_requirements.txt

export PATH="${PROJECT_ROOT}/buffpy/bin:${PATH}"
export PYTHONPATH="${PROJECT_ROOT}/buffpy/lib:${PYTHONPATH}"
