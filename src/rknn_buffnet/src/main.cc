// Copyright (c) 2021 by Rockchip Electronics Co., Ltd. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*-------------------------------------------
                Includes
-------------------------------------------*/
#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>

#include "opencv2/core/core.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "rknn_api.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/time.h>

#include <fstream>
#include <iostream>

#include <chrono>
using namespace std::chrono;

using namespace std;
using namespace cv;


const int MODEL_IN_WIDTH    = 224;
const int MODEL_IN_HEIGHT   = 224;
const int MODEL_IN_CHANNELS = 3;
  
cv::Mat image;
cv::Mat image_rgb;
cv::Mat current_frame;

bool new_frame_available = false;

/*-------------------------------------------
                  Functions
-------------------------------------------*/


void imageCallback(const sensor_msgs::CompressedImageConstPtr& msg)
{
  ROS_INFO("Frame aquired");

  try
  {
    cv::Mat image = cv::imdecode(cv::Mat(msg->data),1);//convert compressed image data to cv::Mat
    // cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
    cv::cvtColor(image, image_rgb, cv::COLOR_BGR2RGB); // convert to rgb

    cv::Mat current_frame = image_rgb.clone();
    if (image.cols != MODEL_IN_WIDTH || image.rows != MODEL_IN_HEIGHT) {
      printf("resize %d %d to %d %d\n", image.cols, image.rows, MODEL_IN_WIDTH, MODEL_IN_HEIGHT);
      cv::resize(image, current_frame, cv::Size(MODEL_IN_WIDTH, MODEL_IN_HEIGHT), (0, 0), (0, 0), cv::INTER_LINEAR);
    }
    new_frame_available = true;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert to image!");
  }
}

static void dump_tensor_attr(rknn_tensor_attr* attr)
{
  printf("  index=%d, name=%s, n_dims=%d, dims=[%d, %d, %d, %d], n_elems=%d, size=%d, fmt=%s, type=%s, qnt_type=%s, "
         "zp=%d, scale=%f\n",
         attr->index, attr->name, attr->n_dims, attr->dims[0], attr->dims[1], attr->dims[2], attr->dims[3],
         attr->n_elems, attr->size, get_format_string(attr->fmt), get_type_string(attr->type),
         get_qnt_type_string(attr->qnt_type), attr->zp, attr->scale);
}

static unsigned char* load_model(const char* filename, int* model_size)
{
  FILE* fp = fopen(filename, "rb");
  if (fp == nullptr) {
    printf("fopen %s fail!\n", filename);
    return NULL;
  }
  fseek(fp, 0, SEEK_END);
  int            model_len = ftell(fp);
  unsigned char* model     = (unsigned char*)malloc(model_len);
  fseek(fp, 0, SEEK_SET);
  if (model_len != fread(model, 1, model_len, fp)) {
    printf("fread %s fail!\n", filename);
    free(model);
    return NULL;
  }
  *model_size = model_len;
  if (fp) {
    fclose(fp);
  }
  return model;
}

static int rknn_GetTop(float* pfProb, float* pfMaxProb, uint32_t* pMaxClass, uint32_t outputCount, uint32_t topNum)
{
  uint32_t i, j;

#define MAX_TOP_NUM 20
  if (topNum > MAX_TOP_NUM)
    return 0;

  memset(pfMaxProb, 0, sizeof(float) * topNum);
  memset(pMaxClass, 0xff, sizeof(float) * topNum);

  for (j = 0; j < topNum; j++) {
    for (i = 0; i < outputCount; i++) {
      if ((i == *(pMaxClass + 0)) || (i == *(pMaxClass + 1)) || (i == *(pMaxClass + 2)) || (i == *(pMaxClass + 3)) ||
          (i == *(pMaxClass + 4))) {
        continue;
      }

      if (pfProb[i] > *(pfMaxProb + j)) {
        *(pfMaxProb + j) = pfProb[i];
        *(pMaxClass + j) = i;
      }
    }
  }

  return 1;
}

/*-------------------------------------------
                  Main Function
-------------------------------------------*/
int main(int argc, char** argv)
{

  rknn_context   ctx;
  int            ret;
  int            model_len = 0;
  unsigned char* model;

  // char* model_ext = "/buffpy/data/models/mobilenet_v1.rknn";
  // char* image_ext = "/data/dog_244x244.jpg";

  const char* model_path = "/home/cu-robotics/buff-code/buffpy/data/models/mobilenet_v1.rknn";
  const char* img_path   = "/home/cu-robotics/buff-code/data/dog_224x224.jpg";
  // strcat(model_path, "/buffpy/data/models/mobilenet_v1.rknn");
  // strcat(img_path, "/data/dog_244x244.jpg");

  // Load RKNN Model
  model = load_model(model_path, &model_len);
  ret   = rknn_init(&ctx, model, model_len, 0, NULL);
  if (ret < 0) {
    printf("rknn_init fail! ret=%d\n", ret);
    return -1;
  }

  // Get Model Input Output Info
  rknn_input_output_num io_num;
  ret = rknn_query(ctx, RKNN_QUERY_IN_OUT_NUM, &io_num, sizeof(io_num));
  if (ret != RKNN_SUCC) {
    printf("rknn_query fail! ret=%d\n", ret);
    return -1;
  }
  printf("model input num: %d, output num: %d\n", io_num.n_input, io_num.n_output);

  printf("input tensors:\n");
  rknn_tensor_attr input_attrs[io_num.n_input];
  memset(input_attrs, 0, sizeof(input_attrs));
  for (int i = 0; i < io_num.n_input; i++) {
    input_attrs[i].index = i;
    ret                  = rknn_query(ctx, RKNN_QUERY_INPUT_ATTR, &(input_attrs[i]), sizeof(rknn_tensor_attr));
    if (ret != RKNN_SUCC) {
      printf("rknn_query fail! ret=%d\n", ret);
      return -1;
    }
    dump_tensor_attr(&(input_attrs[i]));
  }

  printf("output tensors:\n");
  rknn_tensor_attr output_attrs[io_num.n_output];
  memset(output_attrs, 0, sizeof(output_attrs));
  for (int i = 0; i < io_num.n_output; i++) {
    output_attrs[i].index = i;
    ret                   = rknn_query(ctx, RKNN_QUERY_OUTPUT_ATTR, &(output_attrs[i]), sizeof(rknn_tensor_attr));
    if (ret != RKNN_SUCC) {
      printf("rknn_query fail! ret=%d\n", ret);
      return -1;
    }
    dump_tensor_attr(&(output_attrs[i]));
  }
    

  // Set Input Data
  rknn_input inputs[1];
  memset(inputs, 0, sizeof(inputs));
  inputs[0].index = 0;
  inputs[0].type  = RKNN_TENSOR_UINT8;
  inputs[0].size  = MODEL_IN_HEIGHT * MODEL_IN_WIDTH * MODEL_IN_CHANNELS * sizeof(uint8_t);
  inputs[0].fmt   = RKNN_TENSOR_NHWC;


  // Get Output
  rknn_output outputs[1];
  memset(outputs, 0, sizeof(outputs));
  outputs[0].want_float = 0;

  // ROS setup
  ros::init(argc, argv, "buffnet");
  ros::NodeHandle n;
  ros::spinOnce();
  ros::Rate loop_rate(40);
  sleep(4);
  ros::Subscriber sub = n.subscribe("/image_raw/compressed", 1, imageCallback);

  int frames = 1000;
  while (ros::ok()) {

    // Set image data
    if (!new_frame_available) {
      continue;
    }
    ROS_INFO("frame!\n");
    // auto stop = high_resolution_clock::now();
    // auto duration = duration_cast<microseconds>(stop - start);
    // printf("Frames: %i\nElapsed time (microseconds): %f\n", frames, 1e-5 * duration.count());
    // printf("Average FPS: %f\n", 1e5 * frames / duration.count());
    // auto new_frame = high_resolution_clock::now();
    new_frame_available = false;
    inputs[0].buf = current_frame.data;

    ret = rknn_inputs_set(ctx, io_num.n_input, inputs);
    if (ret < 0) {
      printf("rknn_input_set fail! ret=%d\n", ret);
      return -1;
    }

    // Run
    ret = rknn_run(ctx, nullptr);
    if (ret < 0) {
      printf("rknn_run fail! ret=%d\n", ret);
      return -1;
    }

    
    ret = rknn_outputs_get(ctx, 1, outputs, NULL);
    if (ret < 0) {
      printf("rknn_outputs_get fail! ret=%d\n", ret);
      return -1;
    }


    // Post Process
    for (int i = 0; i < io_num.n_output; i++) {
      uint32_t MaxClass[5];
      float    fMaxProb[5];
      float*   buffer = (float*)outputs[i].buf;
      uint32_t sz     = outputs[i].size / 4;

      rknn_GetTop(buffer, fMaxProb, MaxClass, sz, 5);

      printf(" --- Top5 ---\n");
      for (int i = 0; i < 5; i++) {
        printf("%3d: %8.6f\n", MaxClass[i], fMaxProb[i]);
      }
    }
    rknn_outputs_release(ctx, 1, outputs);
    loop_rate.sleep();
  }

  // Release rknn_outputs
  rknn_outputs_release(ctx, 1, outputs);


  // Release
  if (ctx >= 0) {
    rknn_destroy(ctx);
  }
  if (model) {
    free(model);
  }
  return 0;
}
