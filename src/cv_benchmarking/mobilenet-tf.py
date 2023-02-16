#! /usr/bin/python3

import cv2
import time
import numpy as np
import tensorflow as tf
import matplotlib.pyplot as plt

from tensorflow.keras.preprocessing import image
from tensorflow.keras.applications import imagenet_utils

def process_image(img, model, preprocess):
	#initializing the model to predict the image details using predefined models.
	finalimg = preprocess([img])
	predictions = model.predict(finalimg)
	# To predict and decode the image details
	results = imagenet_utils.decode_predictions(predictions)
	return results

def main():

	cam_port = 0
	duration = 10
	prediction_count = 0

	cam = cv2.VideoCapture(cam_port, cv2.CAP_V4L2)
	cam.set(cv2.CAP_PROP_FRAME_WIDTH, 224)
	cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 224)
	cam.set(cv2.CAP_PROP_FPS, 36)

	# model = tf.keras.applications.mobilenet_v2.MobileNetV2()

	start = time.time()

	while time.time() - start < duration:
		# reading the input using the camera
		result, image = cam.read()

		# If image will detected without any error,
		# show result
		if result:
			prediction_count += 1
			# predictions = process_image(image, model, tf.keras.applications.mobilenet_v2.preprocess_input)

		# If captured image is corrupted, moving to else part
		else:
			print("No image detected.")

	print(f"Model PPS: {prediction_count / duration}")




if __name__ =='__main__':
	main()