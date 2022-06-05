#! /bin/bash

# This install was taken from
# https://www.sublimetext.com/docs/linux_repositories.html

# Install GPG key
wget -qO - https://download.sublimetext.com/sublimehq-pub.gpg | sudo apt-key add -

# Select the channel to use, this script uses stable
echo "deb https://download.sublimetext.com/ apt/stable/" | sudo tee /etc/apt/sources.list.d/sublime-text.list

# Update Apt
sudo apt update

# Install dependencies
sudo apt install -y apt-transport-https

# Update apt again
sudo apt update

# Install Sublime
sudo apt install -y sublime-text