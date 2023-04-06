#include "buffnet.h"
#include "buff_realsense.h"

#include <ros/ros.h>
#include <image_transport/image_transport.h>


Buffnet::Buffnet() {
}

void Buffnet::spin_realsense() {
	rs.read_frames();
}


// rknn_context   ctx;
// int            ret;
// int            model_len = 0;
// unsigned char* model;
// const char* model_path = "/home/cu-robotics/buff-code/buffpy/data/models/mobilenet_v1.rknn";
// const char* img_path   = "/home/cu-robotics/buff-code/data/dog_224x224.jpg";

// // Load RKNN Model
// model = load_model(model_path, &model_len);
// ret   = rknn_init(&ctx, model, model_len, 0, NULL);
// if (ret < 0) {
// printf("rknn_init fail! ret=%d\n", ret);
// return -1;
// }

// // Get Model Input Output Info
// rknn_input_output_num io_num;
// ret = rknn_query(ctx, RKNN_QUERY_IN_OUT_NUM, &io_num, sizeof(io_num));
// if (ret != RKNN_SUCC) {
// printf("rknn_query fail! ret=%d\n", ret);
// return -1;
// }
// printf("model input num: %d, output num: %d\n", io_num.n_input, io_num.n_output);

// printf("input tensors:\n");
// rknn_tensor_attr input_attrs[io_num.n_input];
// memset(input_attrs, 0, sizeof(input_attrs));
// for (int i = 0; i < io_num.n_input; i++) {
// input_attrs[i].index = i;
// ret                  = rknn_query(ctx, RKNN_QUERY_INPUT_ATTR, &(input_attrs[i]), sizeof(rknn_tensor_attr));
// if (ret != RKNN_SUCC) {
//   printf("rknn_query fail! ret=%d\n", ret);
//   return -1;
// }
// dump_tensor_attr(&(input_attrs[i]));
// }

// printf("output tensors:\n");
// rknn_tensor_attr output_attrs[io_num.n_output];
// memset(output_attrs, 0, sizeof(output_attrs));
// for (int i = 0; i < io_num.n_output; i++) {
// output_attrs[i].index = i;
// ret                   = rknn_query(ctx, RKNN_QUERY_OUTPUT_ATTR, &(output_attrs[i]), sizeof(rknn_tensor_attr));
// if (ret != RKNN_SUCC) {
//   printf("rknn_query fail! ret=%d\n", ret);
//   return -1;
// }
// dump_tensor_attr(&(output_attrs[i]));
// }


// // Set Input Data
// rknn_input inputs[1];
// memset(inputs, 0, sizeof(inputs));
// inputs[0].index = 0;
// inputs[0].type  = RKNN_TENSOR_UINT8;
// inputs[0].size  = MODEL_IN_HEIGHT * MODEL_IN_WIDTH * MODEL_IN_CHANNELS * sizeof(uint8_t);
// inputs[0].fmt   = RKNN_TENSOR_NHWC;


// // Get Output
// rknn_output outputs[1];
// memset(outputs, 0, sizeof(outputs));
// outputs[0].want_float = 0;

// inputs[0].buf = color.get_data();

// ret = rknn_inputs_set(ctx, io_num.n_input, inputs);
// if (ret < 0) {
//   printf("rknn_input_set fail! ret=%d\n", ret);
//   return -1;
// }

// // Run
// ret = rknn_run(ctx, nullptr);
// if (ret < 0) {
//   printf("rknn_run fail! ret=%d\n", ret);
//   return -1;
// }


// ret = rknn_outputs_get(ctx, 1, outputs, NULL);
// if (ret < 0) {
//   printf("rknn_outputs_get fail! ret=%d\n", ret);
//   return -1;
// }


// // Post Process
// for (int i = 0; i < io_num.n_output; i++) {
//   uint32_t MaxClass[5];
//   float    fMaxProb[5];
//   float*   buffer = (float*)outputs[i].buf;
//   uint32_t sz     = outputs[i].size / 4;

//   rknn_GetTop(buffer, fMaxProb, MaxClass, sz, 5);

//   printf(" --- Top5 ---\n");
//   for (int i = 0; i < 5; i++) {
//     printf("%3d: %8.6f\n", MaxClass[i], fMaxProb[i]);
//   }
// }
// rknn_outputs_release(ctx, 1, outputs);

// // Release rknn_outputs
// rknn_outputs_release(ctx, 1, outputs);


// // Release
// if (ctx >= 0) {
// rknn_destroy(ctx);
// }
// if (model) {
// free(model);
// }


/*-------------------------------------------
                  Functions
-------------------------------------------*/

// static void dump_tensor_attr(rknn_tensor_attr* attr)
// {
//   printf("  index=%d, name=%s, n_dims=%d, dims=[%d, %d, %d, %d], n_elems=%d, size=%d, fmt=%s, type=%s, qnt_type=%s, "
//          "zp=%d, scale=%f\n",
//          attr->index, attr->name, attr->n_dims, attr->dims[0], attr->dims[1], attr->dims[2], attr->dims[3],
//          attr->n_elems, attr->size, get_format_string(attr->fmt), get_type_string(attr->type),
//          get_qnt_type_string(attr->qnt_type), attr->zp, attr->scale);
// }

// static unsigned char* load_model(const char* filename, int* model_size)
// {
//   FILE* fp = fopen(filename, "rb");
//   if (fp == nullptr) {
//     printf("fopen %s fail!\n", filename);
//     return NULL;
//   }
//   fseek(fp, 0, SEEK_END);
//   int            model_len = ftell(fp);
//   unsigned char* model     = (unsigned char*)malloc(model_len);
//   fseek(fp, 0, SEEK_SET);
//   if (model_len != fread(model, 1, model_len, fp)) {
//     printf("fread %s fail!\n", filename);
//     free(model);
//     return NULL;
//   }
//   *model_size = model_len;
//   if (fp) {
//     fclose(fp);
//   }
//   return model;
// }

// static int rknn_GetTop(float* pfProb, float* pfMaxProb, uint32_t* pMaxClass, uint32_t outputCount, uint32_t topNum)
// {
//   uint32_t i, j;

// #define MAX_TOP_NUM 20
//   if (topNum > MAX_TOP_NUM)
//     return 0;

//   memset(pfMaxProb, 0, sizeof(float) * topNum);
//   memset(pMaxClass, 0xff, sizeof(float) * topNum);

//   for (j = 0; j < topNum; j++) {
//     for (i = 0; i < outputCount; i++) {
//       if ((i == *(pMaxClass + 0)) || (i == *(pMaxClass + 1)) || (i == *(pMaxClass + 2)) || (i == *(pMaxClass + 3)) ||
//           (i == *(pMaxClass + 4))) {
//         continue;
//       }

//       if (pfProb[i] > *(pfMaxProb + j)) {
//         *(pfMaxProb + j) = pfProb[i];
//         *(pMaxClass + j) = i;
//       }
//     }
//   }

//   return 1;
// }