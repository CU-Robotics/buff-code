#!/bin/bash

cd /home/cu-robotics/buff-code
source buffpy/buff.bash
ROBOT=$(<whoami.txt)
run ROBOT > /home/cu-robotics/buff.log
