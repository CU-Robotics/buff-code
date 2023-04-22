# Buffpy
Buffpy is two things:

`buffpy/` is a deployable package that contains all the executables and configurations the robot will need.
`buffpy/scripts` contains tools for setting up, building and installing projects into `buffpy/`.


## Development Tools

Build projects with 

        buffpy -b <build_profile>

where a build build profile is a yaml file (filename without .yaml) under `buffpy/data/build`.

These files contains a bash command to build each project and the files that need to be installed. Profiles can also include other profiles (one command to build them all).

Then deploy to a robot

        buffpy -d



### Run Script

Another `buffpy/scripts` python tool to launch our robot's software.

        run <robot_name>

where `robot_name` is a configuration under `buffpy/robots`

Each robot configuration contains the executables that buffpy builds and any other `ros nodes` (TODO link ros wiki) that you want to run.

The configurations also contain program setup data for all `nodes`.

A `node` is a process or thread in the software pipeline. Projects produce executables, run spawns them. You can remove buffpy from the picture by manually bulding all the necesary projects and copying the products to a location known by some process spawner (could be a simple as a bash script, run `./file_path &` to spawn a process in the background).


## Deployable Package

Deploy your builds (that you make using `buffpy -b <build_profile>`) to a robot using

        buffpy -d

This will send the contents of `buffpy/` to what ever IPs are listed in  `buffpy/data/robots/robots.yaml`, the robot registry.

You can install your ssh key to the robot so it doesn't require a password when deploying and sshing.

        buffpy --installKeys

Use the default setup options (or set it up manually, it's not hard).


## Project Clean up 

sus atm

        buffpy --clean all

TODO: Building
        - build cache (for profiles building multiple projects) 
        - figure out projects that need libs from other projects 

TODO: Deploying
        - deploy cache, if the robot has the files don't send them

TODO: Cleaning
        - better base clean implementation (removing buffpy installed items based on build profiles)
        - add project specific clean commands (to clean things in source), these might have to be seperate commands