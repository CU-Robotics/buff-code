#! /usr/bin/env bash

#  run this script with 'source install.bash'

DIR=$PWD

echo -e "Running Install from ${PWD}"
echo -e "\n\tapt updating...\n"

sudo apt update

echo -e "\n\tInstalling Dependencies...\n"

xargs sudo apt install <dependencies.txt -y
pip3 install -r python3_requirements.txt