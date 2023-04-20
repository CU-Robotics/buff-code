# Containers
## Contains software development tools to provide a development environment for developers developing outside of our development environment using the development version of docker compose, specified with \<machine_architecture\> -dev.

Refer to google drive `software/documentation` for more info.

## Docker Image Name
        curobotics/buffbox:aarch64-dev
### Update Image
        docker pull curobotics/buffbox:aarch64-dev
### Run Image
        sudo docker run -it -v <your absolute buff-code directory>:/home/cu-robotics/buff-code curobotics/buffbox:aarch64-dev

## Uses
This is meant to be a standard environment for building (and possibly deploying in the future.)
### Building
        buffpy -b <project>
