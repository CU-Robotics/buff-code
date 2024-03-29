#! /usr/bin/env python3
"""
	Project:
			buffnet detector
	Author: Mitchell D Scott
	Description:
		Detects and displays images
"""
import os
import sys
import cv2
import time
import yaml
import rospy
import torch
import torchvision
import numpy as np
import depthai as dai
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray


def xywh2xyxy(x):
    # Convert nx4 boxes from [x, y, w, h] to [x1, y1, x2, y2] where xy1=top-left, xy2=bottom-right
    y = torch.zeros_like(x) if isinstance(x, torch.Tensor) else np.zeros_like(x)
    y[:, 0] = x[:, 0] - x[:, 2] / 2  # top left x
    y[:, 1] = x[:, 1] - x[:, 3] / 2  # top left y
    y[:, 2] = x[:, 0] + x[:, 2] / 2  # bottom right x
    y[:, 3] = x[:, 1] + x[:, 3] / 2  # bottom right y
    return y


def non_max_suppression(prediction, conf_thres=0.1, iou_thres=0.6, merge=False, classes=None, agnostic=False):
    """Performs Non-Maximum Suppression (NMS) on inference results
    Returns:
         detections with shape: nx6 (x1, y1, x2, y2, conf, cls)
    """
    prediction=torch.from_numpy(prediction)
    if prediction.dtype is torch.float16:
        prediction = prediction.float()  # to FP32

    nc = prediction[0].shape[1] - 5  # number of classes
    xc = prediction[..., 4] > conf_thres  # candidates

    # Settings
    min_wh, max_wh = 2, 4096  # (pixels) minimum and maximum box width and height
    max_det = 300  # maximum number of detections per image
    time_limit = 10.0  # seconds to quit after
    redundant = True  # require redundant detections
    multi_label = nc > 1  # multiple labels per box (adds 0.5ms/img)

    t = time.time()
    output = [None] * prediction.shape[0]
    for xi, x in enumerate(prediction):  # image index, image inference
        # Apply constraints
        # x[((x[..., 2:4] < min_wh) | (x[..., 2:4] > max_wh)).any(1), 4] = 0  # width-height
        x = x[xc[xi]]  # confidence

        # If none remain process next image
        if not x.shape[0]:
            continue

        # Compute conf
        x[:, 5:] *= x[:, 4:5]  # conf = obj_conf * cls_conf

        # Box (center x, center y, width, height) to (x1, y1, x2, y2)
        box = xywh2xyxy(x[:, :4])

        # Detections matrix nx6 (xyxy, conf, cls)
        if multi_label:
            i, j = (x[:, 5:] > conf_thres).nonzero(as_tuple=False).T
            x = torch.cat((box[i], x[i, j + 5, None], j[:, None].float()), 1)
        else:  # best class only
            conf, j = x[:, 5:].max(1, keepdim=True)
            x = torch.cat((box, conf, j.float()), 1)[conf.view(-1) > conf_thres]

        # Filter by class
        if classes:
            x = x[(x[:, 5:6] == torch.tensor(classes, device=x.device)).any(1)]

        # Apply finite constraint
        # if not torch.isfinite(x).all():
        #     x = x[torch.isfinite(x).all(1)]

        # If none remain process next image
        n = x.shape[0]  # number of boxes
        if not n:
            continue

        # Sort by confidence
        # x = x[x[:, 4].argsort(descending=True)]

        # Batched NMS
        c = x[:, 5:6] * (0 if agnostic else max_wh)  # classes
        boxes, scores = x[:, :4] + c, x[:, 4]  # boxes (offset by class), scores
        i = torchvision.ops.boxes.nms(boxes, scores, iou_thres)
        if i.shape[0] > max_det:  # limit detections
            i = i[:max_det]
        if merge and (1 < n < 3E3):  # Merge NMS (boxes merged using weighted mean)
            try:  # update boxes as boxes(i,4) = weights(i,n) * boxes(n,4)
                iou = box_iou(boxes[i], boxes) > iou_thres  # iou matrix
                weights = iou * scores[None]  # box weights
                x[i, :4] = torch.mm(weights, x[:, :4]).float() / weights.sum(1, keepdim=True)  # merged boxes
                if redundant:
                    i = i[iou.sum(1) > 1]  # require redundancy
            except:  # possible CUDA error https://github.com/ultralytics/yolov3/issues/1139
                print(x, i, x.shape, i.shape)
                pass

        output[xi] = x[i]
        if (time.time() - t) > time_limit:
            break  # time limit exceeded

    return output

labelMap = [
	'blue-armor',
	'red-armor',
	'purple-armor',
	'robot'
]

cam_options = ['rgb', 'left', 'right']


def draw_boxes(image, labels):
	if labels is None or len(labels) == 0:
		return image
	else:
		colors = [(255,0,0), (0,0,255), (255,0,255), (0,255,0)]

		for c,cf,x,y,w,h in labels:
			# Draw a rectangle on the image
			image = cv2.rectangle(image, (int(x - (w/2)), int(y - (h/2))), (int(x + (w/2)), int(y + (h/2))), colors[int(c)], 2)

			image = cv2.rectangle(image, (int(x - (w/2)), int(y - (h/2)) - 20), (int(x - (w/2)) + 75, int(y - (h/2))), colors[int(c)], -1)
			cv2.putText(image, f'{int(c)}: {cf:.2f}', (int(x - (w/2)), int(y - (h/2))), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 1)

	return image


class DepthAI_Device:
	def __init__(self):
		"""
				Define all the parameters of the model here.
				Can be initialized with a config file, a system launch
				or manually from a terminal. will exit if not enough params
				exist.
		"""
		# It is assumed that this script will only be called by the ols
		# If the ols is used properly these will always be defined
		topics = rospy.get_param('/buffbot/TOPICS')
		self.debug = rospy.get_param('/buffbot/DEBUG')
		self.iou = rospy.get_param('/buffbot/MODEL/IOU')
		self.FPS = rospy.get_param('/buffbot/CAMERA/FPS')
		self.confidence = rospy.get_param('/buffbot/MODEL/CONFIDENCE')
		self.image_size = rospy.get_param('/buffbot/CAMERA/RESOLUTION')

		model_dir = os.path.join(os.getenv('PROJECT_ROOT'), 'buffpy', 'models')
		model_path = os.path.join(model_dir, rospy.get_param('/buffbot/MODEL/BLOB_FILE'))

		self.bridge = CvBridge()

		# Start defining a pipeline
		self.init_depthai_pipeline(model_path)

		rospy.init_node('buffnet', anonymous=True)
		self.det_pub = rospy.Publisher(
			topics['DETECTION_PIXEL'], Float64MultiArray, queue_size=1)

		self.red_pub = rospy.Publisher(
			topics['DETECTION_RED'], Float64MultiArray, queue_size=1)

		self.blue_pub = rospy.Publisher(
			topics['DETECTION_BLUE'], Float64MultiArray, queue_size=1)

		self.image_pub = rospy.Publisher(
			topics['IMAGE'], Image, queue_size=1)

		if self.debug:
			self.ann_pub = rospy.Publisher(
				topics['IMAGE_DEBUG'], Image, queue_size=1)


	def init_depthai_pipeline(self, model_path):
		self.pipeline = dai.Pipeline()

		self.pipeline.setOpenVINOVersion(version=dai.OpenVINO.VERSION_2021_4)

		# Define a neural network that will make predictions based on the source frames
		detection_nn = self.pipeline.create(dai.node.NeuralNetwork)
		detection_nn.setBlobPath(model_path)

		detection_nn.setNumPoolFrames(4)
		detection_nn.input.setBlocking(False)
		detection_nn.setNumInferenceThreads(2)

		# Define a source - color camera
		cam = self.pipeline.create(dai.node.ColorCamera)
		cam.setPreviewSize(self.image_size, self.image_size)
		cam.setInterleaved(False)
		cam.preview.link(detection_nn.input)
		cam.setFps(self.FPS)

		# Create outputs
		xout_rgb = self.pipeline.create(dai.node.XLinkOut)
		xout_rgb.setStreamName("nn_input")
		xout_rgb.input.setBlocking(False)
		detection_nn.passthrough.link(xout_rgb.input)

		xout_nn = self.pipeline.create(dai.node.XLinkOut)
		xout_nn.setStreamName("nn")
		xout_nn.input.setBlocking(False)

		detection_nn.out.link(xout_nn.input)

		# Define a camera control stream
		controlIn = self.pipeline.create(dai.node.XLinkIn)
		controlIn.out.link(cam.inputControl)
		controlIn.setStreamName('control')


	def spin(self):

		with dai.Device(self.pipeline, usb2Mode=True) as device:

			if 1:
				device.setLogLevel(dai.LogLevel.TRACE)
				device.setLogOutputLevel(dai.LogLevel.TRACE)

			q_nn_input = device.getOutputQueue(
				name="nn_input", maxSize=4, blocking=False)
			q_nn = device.getOutputQueue(
				name="nn", maxSize=4, blocking=False)

			controlQueue = device.getInputQueue('control')
			ctrl = dai.CameraControl()
			# ctrl.setContrast(10)
			# ctrl.setBrightness(-5)
			# ctrl.setSaturation(10)
			# ctrl.setAutoWhiteBalanceMode(dai.RawCameraControl.AutoWhiteBalanceMode.OFF)
			ctrl.setAutoFocusMode(dai.RawCameraControl.AutoFocusMode.OFF)
			ctrl.setManualFocus(100)
			controlQueue.send(ctrl)

			start_time = time.time()
			layer_info_printed = False

			rospy.loginfo("Starting the Stream")

			while not rospy.is_shutdown():
				in_nn_input = q_nn_input.get()
				in_nn = q_nn.get()

				frame = in_nn_input.getCvFrame()

				# get the "output" layer
				try:
					output = np.array(in_nn.getLayerFp16("output"))
				except:
					rospy.loginfo("getLayerFp16 Error occured, retrying")
					continue

				# reshape to proper format
				cols = output.shape[0]//6300
				output = np.reshape(output, (6300, cols))
				output = np.expand_dims(output, axis=0)

				total_classes = cols - 5

				boxes = non_max_suppression(output, conf_thres=self.confidence, iou_thres=self.iou)

				boxes = boxes[0]

				reds = []
				blues = []
				labels = []
				detections = []

				if not boxes is None:
					boxes.numpy()
					for x1,y1,x2,y2,cf,cl in boxes:
						x = int((x1 + x2) / 2) 
						y = int((y1 + y2) / 2)
						w = abs(x2 - x1)
						h = abs(y2 - y1)

						if cl == 0:
							blues = np.concatenate([blues, 
								[x / self.image_size,
								y / self.image_size,
								w / self.image_size,
								h / self.image_size]])
						elif cl == 1:
							reds = np.concatenate([reds, 
								[x / self.image_size,
								y / self.image_size,
								w / self.image_size,
								h / self.image_size]])

						labels.append([cl,cf,x,y,w,h])

					if len(reds) > 1:
						msg = Float64MultiArray(data=reds)
						self.red_pub.publish(msg)
					elif len(blues) > 1:
						msg = Float64MultiArray(data=blues)
						self.blue_pub.publish(msg)

				image_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
				self.image_pub.publish(image_msg)
					
				if self.debug:
					ann_frame = draw_boxes(frame, labels)
					ann_img_msg = self.bridge.cv2_to_imgmsg(ann_frame, "bgr8")
					self.ann_pub.publish(ann_img_msg)

def main(name):
	device = DepthAI_Device()
	try:
		device.spin()
	except Exception as e:
		rospy.logerr(e)
		exit(1)


if __name__ == '__main__':
	if len(sys.argv) > 1:
		main(sys.argv[1])



		