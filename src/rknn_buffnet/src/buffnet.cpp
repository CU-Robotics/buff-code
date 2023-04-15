#include "buffnet.h"
#include "buff_realsense.h"


static void dump_tensor_attr_ros(rknn_tensor_attr* attr) {
	ROS_INFO("index=%d, name=%s, n_dims=%d, dims=[%d, %d, %d, %d], n_elems=%d, size=%d, fmt=%s, type=%s, qnt_type=%s, "
		"zp=%d, scale=%f",
		attr->index, attr->name, attr->n_dims, attr->dims[0], attr->dims[1], attr->dims[2], attr->dims[3],
		attr->n_elems, attr->size, get_format_string(attr->fmt), get_type_string(attr->type),
		get_qnt_type_string(attr->qnt_type), attr->zp, attr->scale);
}

static unsigned char* load_model(const char* filename, int* model_size) {
	FILE* fp = fopen(filename, "rb");
	if (fp == nullptr) {
		ROS_ERROR("fopen %s fail!\n", filename);
		return NULL;
	}
	
	fseek(fp, 0, SEEK_END);
	
	int model_len = ftell(fp);
	unsigned char* model = (unsigned char*)malloc(model_len);
	
	fseek(fp, 0, SEEK_SET);

	if (model_len != fread(model, 1, model_len, fp)) {
		ROS_ERROR("fread %s fail!\n", filename);
		free(model);
		return NULL;
	}
	
	*model_size = model_len;

	if (fp) {
		fclose(fp);
	}
	
	return model;
}

static int post_process(float* output, uint32_t outputCount) {

	ROS_INFO("Output count: %i", outputCount);
	for (int i = 0; i < outputCount; i+=4) {
		ROS_INFO("output value %i: %f", i, output[i]);
			// if ((i == *(pMaxClass + 0)) || (i == *(pMaxClass + 1)) || (i == *(pMaxClass + 2)) || (i == *(pMaxClass + 3)) ||
			// 	(i == *(pMaxClass + 4))) {
			// 	continue;
			// }

			// if (pfProb[i] > *(pfMaxProb + j)) {
			// 	*(pfMaxProb + j) = pfProb[i];
			// 	*(pMaxClass + j) = i;
			// }
	}

  return 1;
}


Buffnet::Buffnet() {
	model_len = 0;
	char* model_path = std::getenv("PROJECT_ROOT");
	strcat(model_path,"/buffpy/data/models/buffnet.rknn");
	ROS_INFO("Model path: %s", model_path);

	model = load_model(model_path, &model_len);
	ret   = rknn_init(&ctx, model, model_len, 0, NULL);
	if (ret < 0) {
		ROS_ERROR("rknn_init fail! ret=%d\n", ret);
		model_active = 0;
	}
	model_info();
	init_model();
}

Buffnet::~Buffnet() {
	// Release
	if (ctx >= 0) {
		rknn_destroy(ctx);
	}
	if (model) {
		free(model);
	}
}

void Buffnet::model_info() {
	// // Get Model Input Output Info
	ret = rknn_query(ctx, RKNN_QUERY_IN_OUT_NUM, &io_num, sizeof(io_num));
	if (ret != RKNN_SUCC) {
		model_active = 0;
		ROS_ERROR("rknn_query fail! ret=%d\n", ret);
	}
	else {
		model_active = 1;
		ROS_INFO("BuffNet input num: %d, output num: %d", io_num.n_input, io_num.n_output);
		rknn_tensor_attr input_attrs[io_num.n_input];
		memset(input_attrs, 0, sizeof(input_attrs));
		for (int i = 0; i < io_num.n_input; i++) {
			input_attrs[i].index = i;
			ret = rknn_query(ctx, RKNN_QUERY_INPUT_ATTR, &(input_attrs[i]), sizeof(rknn_tensor_attr));

			if (ret != RKNN_SUCC) {
				model_active = 0;
				ROS_ERROR("rknn_query fail! ret=%d\n", ret);
			}
			else {
				dump_tensor_attr_ros(&(input_attrs[i]));
			}
		}
	}
}

void Buffnet::init_model() {
	// Set Input Data
	memset(inputs, 0, sizeof(inputs));
	inputs[0].index = 0;
	inputs[0].type  = RKNN_TENSOR_UINT8;
	inputs[0].size  = MODEL_IN_HEIGHT * MODEL_IN_WIDTH * MODEL_IN_CHANNELS * sizeof(uint8_t);
	inputs[0].fmt   = RKNN_TENSOR_NHWC;

	// Get Output
	memset(outputs, 0, sizeof(outputs));
	outputs[0].want_float = NPU_USE_FLOATS;
}

int Buffnet::spin_model() {
	inputs[0].buf = rs.get_color_cv_resized(MODEL_IN_WIDTH, MODEL_IN_HEIGHT).data;

	ret = rknn_inputs_set(ctx, io_num.n_input, inputs);
	if (ret < 0) {
		model_active = 0;
	 	ROS_ERROR("rknn_input_set fail! ret=%d\n", ret);
	 	return 0;
	}

	// Run
	ret = rknn_run(ctx, nullptr);
	if (ret < 0) {
		model_active = 0;
		ROS_ERROR("rknn_run fail! ret=%d\n", ret);
		return 0;
	}

	rknn_wait(ctx, NULL);
	ret = rknn_outputs_get(ctx, 1, outputs, NULL);
	if (ret < 0) {
		model_active = 0;
		ROS_ERROR("rknn_outputs_get fail! ret=%d\n", ret);
		return 0;
	}

	// // Post Process
	// uint32_t MaxClass[5];
	// float    fMaxProb[5];
	float* buffer = (float*)outputs[0].buf;

	post_process((float*)outputs[0].buf, outputs[0].size / 4);

	rknn_outputs_release(ctx, 1, outputs);

	return 1;
}

void Buffnet::spin_realsense() {
	rs.read_frames();
}

cv::Mat Buffnet::get_depth() {
	return rs.get_depth_cv();
}

cv::Mat Buffnet::get_color() {
	return rs.get_color_cv();
}

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