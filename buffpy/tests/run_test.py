#!/usr/bin/env python3

import os
from robot_description import Robot_Description
from robot_spawner import Robot_Spawner


def main():
	rs = Robot_Spawner()
	rs.spin("penguin")

if __name__ == '__main__':
	main()