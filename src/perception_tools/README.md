## Build perception_tools
        buffpy -b ptools

## Add Training Data
-  From Roboflow, use YOLOv5 export Pytorch export format -> zip for your training data.
- Access from team google drive: software/2023_ml_release/M1/master.zip
- Move zip file to buff-code/data/

## Run Training
        buffpy --train buffnet

## Run Models TODO
Models located in `buff-code/data/models`

Reference in robot located in `~/edge2-npu/C++/yolov5/install/yolov5`

        sudo ./yolov5 data/models/<model>.rknn data/img/<img_name>.jpg