# Buffpy
Buffpy is two things:

## Development Tool
Scripts directory contains build tools.

Run them with `buffpy -b <build_profile>`

where a build build profile is a yaml file (filename without .yaml) under buffpy/data/build.

These files contains a bash command to build its specific project and the files that need to be installed.

### Run Script
Another buffpy python tool to run our robot's software.

        run <robot_name>

where `robot_name` is a configuration under `buffpy/robots`

Each robot configuration contains the executables that buffpy builds and other `ros nodes` (TODO link ros wiki) that you want to run.

The configurations also contain program setup data for all `nodes`.

A `node` is a process or thread in the software pipeline.

## Deployable Package
`buffpy/` is a directory containing configuration and programs to run the robot.

The buffpy python tool deploys this package to robots registered in `buffpy/data/robots/robots.yaml`

