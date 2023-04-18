#include "buffnet.h"
#include "postprocess.h"
#include "buff_realsense.h"

static void dump_tensor_attr_ros(rknn_tensor_attr* attr) {
	ROS_INFO("index=%d, name=%s, n_dims=%d, dims=[%d, %d, %d, %d], n_elems=%d, size=%d, fmt=%s, type=%s, qnt_type=%s, "
		"zp=%d, scale=%f",
		attr->index, attr->name, attr->n_dims, attr->dims[0], attr->dims[1], attr->dims[2], attr->dims[3],
		attr->n_elems, attr->size, get_format_string(attr->fmt), get_type_string(attr->type),
		get_qnt_type_string(attr->qnt_type), attr->zp, attr->scale);
}

double __get_us(struct timeval t) { return (t.tv_sec * 1000000 + t.tv_usec); }

static unsigned char* load_data(FILE* fp, size_t ofst, size_t sz)
{
  	unsigned char* data;
  	int            ret;

  	data = NULL;

  	if (NULL == fp) {
		return NULL;
  	}

  	ret = fseek(fp, ofst, SEEK_SET);
  	if (ret != 0) {
		printf("blob seek failure.\n");
		return NULL;
  	}

  	data = (unsigned char*)malloc(sz);
  	if (data == NULL) {
		printf("buffer malloc failure.\n");
		return NULL;
  	}
  	ret = fread(data, 1, sz, fp);
  	return data;
}

static unsigned char* load_model(const char* filename, int* model_size)
{
  	FILE*          fp;
  	unsigned char* data;

  	fp = fopen(filename, "rb");
  	if (NULL == fp) {
		printf("Open file %s failed.\n", filename);
		return NULL;
  	}

  	fseek(fp, 0, SEEK_END);
  	int size = ftell(fp);

  	data = load_data(fp, 0, size);

  	fclose(fp);

  	*model_size = size;
  	return data;
}

static int saveFloat(const char* file_name, float* output, int element_size)
{
  	FILE* fp;
  	fp = fopen(file_name, "w");
  	for (int i = 0; i < element_size; i++) {
		fprintf(fp, "%.6f\n", output[i]);
  	}
  	fclose(fp);
  	return 0;
}

static int post_process(float* output, uint32_t outputCount) {

  return 1;
}


Buffnet::Buffnet() {
	model_len = 0;
	img_width = MODEL_IN_WIDTH;
	img_height = MODEL_IN_HEIGHT;
	img_channel = MODEL_IN_CHANNELS;
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

		memset(output_attrs, 0, sizeof(output_attrs));
		for (int i = 0; i < io_num.n_output; i++) {
			output_attrs[i].index = i;
			ret = rknn_query(ctx, RKNN_QUERY_OUTPUT_ATTR, &(output_attrs[i]), sizeof(rknn_tensor_attr));
			dump_tensor_attr_ros(&(output_attrs[i]));
		}
	}
}

void Buffnet::init_model() {

  	if (input_attrs[0].fmt == RKNN_TENSOR_NCHW) {
		ROS_INFO("model is NCHW input fmt");
		channel = input_attrs[0].dims[1];
		width   = input_attrs[0].dims[2];
		height  = input_attrs[0].dims[3];
  	} else {
		ROS_INFO("model is NHWC input fmt");
		width   = input_attrs[0].dims[1];
		height  = input_attrs[0].dims[2];
		channel = input_attrs[0].dims[3];
  	}

	// Set Input Data
	memset(inputs, 0, sizeof(inputs));
	inputs[0].index = 0;
	inputs[0].type  = RKNN_TENSOR_UINT8;
	inputs[0].size  = MODEL_IN_HEIGHT * MODEL_IN_WIDTH * MODEL_IN_CHANNELS * sizeof(uint8_t);
	inputs[0].fmt   = RKNN_TENSOR_NHWC;
	inputs[0].pass_through = 0;

	// Get Output
  	memset(outputs, 0, sizeof(outputs));
  	for (int i = 0; i < io_num.n_output; i++) {
		outputs[i].want_float = 0;
  	}
}

int Buffnet::spin_model() {

	inputs[0].buf = (void*)rs.get_color_cv_resized(MODEL_IN_WIDTH, MODEL_IN_HEIGHT).data;

	if (inputs[0].buf == NULL) {
		ROS_ERROR("No image");
		return 0;
	}
	
	ROS_INFO("DEBUG_MARK 1");

	ret = rknn_inputs_set(ctx, io_num.n_input, inputs);
	if (ret < 0) {
		model_active = 0;
	 	ROS_ERROR("rknn_input_set fail! ret=%d\n", ret);
	 	return 0;
	}

	ROS_INFO("DEBUG_MARK 2");

	// Run
	ret = rknn_run(ctx, nullptr);
	if (ret < 0) {
		model_active = 0;
		ROS_ERROR("rknn_run fail! ret=%d\n", ret);
		return 0;
	}

	ROS_INFO("DEBUG_MARK 3");

	rknn_wait(ctx, NULL);
	ret = rknn_outputs_get(ctx, 1, outputs, NULL);
	if (ret < 0) {
		model_active = 0;
		ROS_ERROR("rknn_outputs_get fail! ret=%d\n", ret);
		return 0;
	}

	// // Post Process
	// float* buffer = (float*)outputs[0].buf;
	// float scale_w = (float)width / img_width;
  	// float scale_h = (float)height / img_height;

	// detect_result_group_t detect_result_group;
  	// std::vector<float>    out_scales;
  	// std::vector<int32_t>  out_zps;
  	// for (int i = 0; i < io_num.n_output; ++i) {
	// 	out_scales.push_back(output_attrs[i].scale);
	// 	out_zps.push_back(output_attrs[i].zp);
  	// }
  	// post_process((int8_t*)outputs[0].buf, (int8_t*)outputs[1].buf, (int8_t*)outputs[2].buf, height, width,
	// 		box_conf_threshold, nms_threshold, scale_w, scale_h, out_zps, out_scales, &detect_result_group);

  	// // Draw Objects
  	// char text[256];
  	// for (int i = 0; i < detect_result_group.count; i++) {
	// 	detect_result_t* det_result = &(detect_result_group.results[i]);
	// 	sprintf(text, "%s %.1f%%", det_result->name, det_result->prop * 100);
	// 	printf("%s @ (%d %d %d %d) %f\n", det_result->name, det_result->box.left, det_result->box.top,
	//  			det_result->box.right, det_result->box.bottom, det_result->prop);
	// 	int x1 = det_result->box.left;
	// 	int y1 = det_result->box.top;
	// 	int x2 = det_result->box.right;
	// 	int y2 = det_result->box.bottom;
	// 	rectangle(annotated_img, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(255, 0, 0, 255), 3);
	// 	putText(annotated_img, text, cv::Point(x1, y1 + 12), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));
  	// }

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

cv::Mat Buffnet::get_annot() {
	return annotated_img;
}