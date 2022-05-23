#! /usr/bin/env python3
import os
import sys
import cv2
import yaml
import glob
import string
import random
import numpy as np
import buffvision as bv


def crop_image(image, x, y, w, h):
	h2 = h / 2
	w2 = w / 2
	x1 = int((x - w2) * image.shape[1])
	x2 = int((x + w2) * image.shape[1])
	y1 = int((y - h2) * image.shape[0])
	y2 = int((y + h2) * image.shape[0])

	return image[y1:y2,x1:x2]

def load_backgrounds(data_path):
	images = []
	for file in glob.glob(os.path.join(data_path, '..', 'background', '*')):
		image = cv2.imread(file)
		if image.shape[0] > 320 and image.shape[1] > 320:
			for x in np.linspace(0, image.shape[1] - 320, 3):
				for y in np.linspace(0, image.shape[0] - 320, 3):
					images.append(image[int(y):int(y+320),int(x):int(x+320)])

		else:
			images.append(cv2.resize(image, (320, 320)))

	return images

def generate_images(image, c, x, y, w, h, backgrounds):

	n_samples = 3
	augmented_images = []
	augmented_labels = []

	low = np.array([0, 95, 35])
	high = np.array([80, 255, 255])

	cropped = crop_image(image, x, y, w, h)
	filtered = cv2.GaussianBlur(cropped.copy(), [9,9], 0.6, 0)
	hsv_image = cv2.cvtColor(filtered, cv2.COLOR_BGR2HSV)

	mask = cv2.bitwise_not(cv2.inRange(hsv_image, low, high))
	rgb_mask = cv2.bitwise_and(cropped.copy(), cropped.copy(), mask=mask)
	mask_h,mask_w,mask_c = rgb_mask.shape

	while mask_h > 40:
		mask_shape = np.array(mask.shape) * (0.3 + np.random.rand())
		mask = cv2.resize(mask, mask_shape.astype(int))
		rgb_mask = cv2.resize(rgb_mask, mask_shape.astype(int))
		mask_h,mask_w,mask_c = rgb_mask.shape

		background_idx = (len(backgrounds) - 1) * np.random.rand(4)

		for i in background_idx:
			background = backgrounds[int(i)]
			for x in np.linspace(0, background.shape[1] - mask_w, n_samples):
				for y in np.linspace(0, background.shape[0] - mask_h, n_samples):

					x = max(0, min(background.shape[1] - mask_w, x + (np.random.rand() - 0.5) * (background.shape[1] - mask_w) / 3))
					y = max(0, min(background.shape[0] - mask_h, y + (np.random.rand() - 0.5) * (background.shape[1] - mask_w) / 3))

					zero_mask = np.zeros(background.shape[:2], dtype=np.uint8)
					zero_mask[round(y):round(y+mask_h),round(x):round(x+mask_w)] = mask
					inv_zero_mask = cv2.bitwise_not(zero_mask)

					object_image = np.zeros(background.shape, dtype=np.uint8)
					object_image[int(y):int(y+mask_h),int(x):int(x+mask_w)] = rgb_mask
					background_mask = cv2.bitwise_and(background, background, mask=inv_zero_mask)
					augmented_image = cv2.add(background_mask, object_image)
					label = [[0, x + (mask_w/2), y + (mask_h/2), mask_w, mask_h]]

					augmented_images.append(augmented_image)
					augmented_labels.append(label)

					object_image = np.zeros(background.shape, dtype=np.uint8)
					object_image[int(y):int(y+mask_h),int(x):int(x+mask_w)] = cv2.cvtColor(rgb_mask, cv2.COLOR_BGR2RGB)
					background_mask = cv2.bitwise_and(background, background, mask=inv_zero_mask)
					augmented_image = cv2.add(background_mask, object_image)
					label = [[1, x + (mask_w/2), y + (mask_h/2), mask_w, mask_h]]

					augmented_images.append(augmented_image)
					augmented_labels.append(label)

	return augmented_images, augmented_labels



def main(data_dir):
	project_root = os.getenv('PROJECT_ROOT')
	data_path = os.path.join(project_root, 'data', data_dir)

	gen_path = os.path.join(project_root, 'data', 'Generated')
	train_path = os.path.join(gen_path, 'train')
	valid_path = os.path.join(gen_path, 'valid')

	bv.clear_generated(gen_path)
	bv.make_generated(gen_path)
	
	backgrounds = load_backgrounds(data_path)

	print(f'Backgrounds: {len(backgrounds)}')

	generated_samples = -1

	m = len(data_path) + len('labels') + 2
	label_paths = glob.glob(os.path.join(data_path, 'labels', '*.txt'))
	
	for labelf in label_paths:
		imfile = os.path.join(data_path, 'images', labelf[m:-4] + '.jpg')
		if os.path.exists(imfile):
			image = cv2.imread(imfile)
			[[c, x, y, w, h]] = bv.load_label(labelf)

			if generated_samples == -1:
				print(f'Data samples: {len(label_paths)}')
				print(f'Image shape: {image.shape}')

			images, labels = generate_images(image, c, x, y, w, h, backgrounds)

			print('Saving batch')
			bv.save_txt_label_data(images, labels, gen_path)

	print(f'Generated {generated_samples} images and labels')
		

if __name__ == '__main__':
	if len(sys.argv) > 1:
		main(sys.argv[1])
