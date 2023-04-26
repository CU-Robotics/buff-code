set -e

GCC_COMPILER=aarch64-linux-gnu

#export LD_LIBRARY_PATH=${TOOL_CHAIN}/lib64:$LD_LIBRARY_PATH
export CC=${GCC_COMPILER}-gcc
export CXX=${GCC_COMPILER}-g++

ROOT_PWD=$( cd "$( dirname $0 )" && cd -P "$( dirname "$SOURCE" )" && pwd )

# build
BUILD_DIR=${ROOT_PWD}/build/build_linux_aarch64

mkdir -p ${BUILD_DIR}
cd ${BUILD_DIR}
cmake ../.. -DTARGET_NAME=buffnet
make -j4
make install
cd -
