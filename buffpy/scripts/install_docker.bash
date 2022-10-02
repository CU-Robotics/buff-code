#! /bin/bash

sudo apt-get remove docker docker-engine docker.io containerd runc

sudo apt update

sudo apt -y install \
	ca-certificates \
	curl \
	gnupg \
	lsb-release \
	apt-transport-https \
	software-properties-common

curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -
sudo add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu `lsb_release -cs` test"
sudo apt update
sudo apt -y install docker-ce


sudo apt update

sudo apt -y install docker-ce docker-ce-cli containerd.io docker-compose-plugin

docker run hello-world

sudo apt -y install qemu binfmt-support qemu-user-static

docker run --rm --privileged multiarch/qemu-user-static --reset -p yes
