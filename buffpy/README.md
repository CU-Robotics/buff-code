# Buffpy
Buffpy is two things:

`buffpy/` is a deployable package that contains all the executables and configurations our robots will need.
`buffpy/src` contains tools for setting up, building and installing projects into `buffpy/`.


## Development Tools

Buff-Code uses build profiles to generate executables (nodes). A build build profile is a yaml file (filename without .yaml) under `buffpy/data/build`

Each build profile contains a directory containing source code and bash commands to initialize, build and install the project.

Build projects with 

        buffpy --build <build_profile>

        buffpy -b <build_profile>

Clean Projects with

        buffpy --clean <build_profile>

        buffpy -c <build_profile>

Buffpy can also deploy itself to a robot over WiFi

        buffpy --deploy

        buffpy -d


### Run

Another `buffpy/src` python tool to launch our robot's software.

        run <robot_name>

where `robot_name` is a configuration under `buffpy/robots`

Each robot configuration contains the executables that buffpy builds and any other `ros nodes` (TODO link ros wiki) that you want to run.

The configurations also contain program setup data for all `nodes` and firmware.

A `node` is a process or thread in the software pipeline. Projects produce executables, run spawns them. You can remove buffpy from the picture by manually bulding all the necesary projects and copying the products to a location known by some process spawner (could be a simple as a bash script, run `./file_path &` to spawn a process in the background).


## Deployable Package

Deploy your builds (that you make using `buffpy -b <build_profile>`) to a robot using

        buffpy -d

This will send the contents of `buffpy/` to what ever IPs are listed in  `buffpy/data/robots/robots.yaml`, the robot registry.


### SSH

<details>
<summary>Instructions to generate an SSH key to work with our robot deployment </summary>

#### Generate SSH Key
        ssh-keygen -t ed25519 -C "your_email@example.com"
##### If you set a custom name for your key, configure `~/.ssh/config` to recognize it. More info: https://www.howtogeek.com/devops/how-to-manage-an-ssh-config-file-in-windows-linux/
For a custom named key to work with our scripts, add the below within `~/.ssh/config`. (edgek.local is the current robot ip when connected to its hotspot.)

        Host edgek.local
                HostName edgek.local
                <recommend adding "PreferredAuthentications publickey" here after completing ssh-copy-id>
                IdentityFile ~/.ssh/<your_private_key_file>

Start your ssh-agent:

        eval "$(ssh-agent -s)"

Add your private ssh key to your ssh-agent:

        ssh-add ~/.ssh/<your_private_key_file>
#### Install SSH Key to Robot
You can install your ssh key to the robot so it doesn't require a password when deploying and sshing.

        buffpy --installKeys

or manually with:

        ssh-copy-id -i ~/.ssh/<your_public_key_file> cu-robotics@edgek.local

#### Connect to Robot
While on its hotspot, you can use any ssh-related commands.

        ssh cu-robotics@edgek.local

</details>

## Project Clean up 

TODO: Building
        - build cache (for profiles building multiple projects) 
        - figure out projects that need libs from other projects 

TODO: Deploying
        - deploy cache, if the robot has the files don't send them
