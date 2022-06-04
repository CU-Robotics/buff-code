#!/bin/bash

cd ${PROJECT_ROOT}/..
if [[ ! -d "l_openvino_toolkit_p_2021.4.582" ]]; then
    sudo apt-get install -y pciutils cpio
    sudo apt autoremove

    wget https://github.com/PINTO0309/tflite2tensorflow/releases/download/v1.10.4/l_openvino_toolkit_p_2021.4.582.tgz

    ## install openvino
    tar xf "l_openvino_toolkit_p_2021.4.582.tgz"
    cd l_openvino_toolkit_p_2021.4.582/

    ./install_openvino_dependencies.sh && \
        sed -i 's/decline/accept/g' silent.cfg && \
        ./install.sh --silent silent.cfg

    rm -rf "../l_openvino_toolkit_p_2021.4.582.tgz"

    /opt/intel/openvino_2021/deployment_tools/model_optimizer/install_prerequisites/install_prerequisites.sh
    python3 -m pip install boto3==1.17.39 blobconverter==1.3.0
fi

cd ${PROJECT_ROOT}