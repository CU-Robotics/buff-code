#! /bin/bash

sudo cp /home/cu-robotics/buff-code/buffpy/scripts/buffbot.service /etc/systemd/system

sudo systemctl enable buffbot.service
