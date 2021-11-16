"""
		NN Detector
	Nina's analytical solution: color/blop
	detection approach to CV detection problem.

"""

class NN_Detector:
	def __init__(self):
		"""
			Define all the parameters of the model here.
			No need to save images or results
		"""


	def detect(self, image):
		"""
			Define all image processing operations here
			@PARAMS:
				image: an RGB image (should be 640x480)
			@RETURNS:
				bounds: bounding box of the detected object [(x1,y1), (x2,y2)]
		"""

		return bounds

	def detect_and_publish(self, image, topic='detected_boundary'):
		"""
			Define all image processing operations here.
			Publishes the bounds to topic instead of returning.
			Maybe in debug also publishes an annotated image.
			@PARAMS:
				image: an RGB image (should be 640x480)
			@RETURNS:
				None
		"""

		