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

def augment_image(image, c, x, y, w, h, backgrounds):

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
		mask_shape = np.array(mask.shape) * 0.4
		mask = cv2.resize(mask, mask_shape.astype(int))
		rgb_mask = cv2.resize(rgb_mask, mask_shape.astype(int))
		mask_h,mask_w,mask_c = rgb_mask.shape

		for background in backgrounds:
			for x in np.linspace(0, background.shape[1] - mask_w, n_samples):
				for y in np.linspace(0, background.shape[0] - mask_h, n_samples):

					x = max(0, min(background.shape[1] - mask_w, x + np.random.rand() * 30))
					y = max(0, min(background.shape[0] - mask_h, y + np.random.rand() * 30))

					zero_mask = np.zeros(background.shape[:2], dtype=np.uint8)
					zero_mask[int(y):int(y+mask_h),int(x):int(x+mask_w)] = mask
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

def save_generated_data(images, labels, n):
	key = n
	train_path = os.path.join(os.getenv('PROJECT_ROOT'), 'data', 'Generated', 'train')
	valid_path = os.path.join(os.getenv('PROJECT_ROOT'), 'data', 'Generated', 'valid')

	for i, (image,label) in enumerate(zip(images,labels)):
		key += 1
		if np.random.rand() > 0.9:
			image_path = os.path.join(valid_path, 'images', f'{key}.jpg')
			cv2.imwrite(image_path, image)
			label_path = os.path.join(valid_path, 'labels', f'{key}.txt')
			with open(label_path, 'w') as f:
				for item in label:
					f.write(f'{item[0]} {item[1]/image.shape[1]} {item[2]/image.shape[0]} {item[3]/image.shape[1]} {item[4]/image.shape[0]}\n')
			
		else:
			image_path = os.path.join(train_path, 'images', f'{key}.jpg')
			cv2.imwrite(image_path, image)
			label_path = os.path.join(train_path, 'labels', f'{key}.txt')
			with open(label_path, 'w') as f:
				for item in label:
					f.write(f'{item[0]} {item[1]/image.shape[1]} {item[2]/image.shape[0]} {item[3]/image.shape[1]} {item[4]/image.shape[0]}\n')
			
	return key

def main(data_dir):
	project_root = os.getenv('PROJECT_ROOT')
	data_path = os.path.join(project_root, 'data', data_dir)

	train_path = os.path.join(data_path, 'Generated', 'train')
	valid_path = os.path.join(data_path, 'Generated', 'valid')

	for file in glob.glob(os.path.join(train_path, 'images', '*.jpg')):
		os.remove(file)

	for file in glob.glob(os.path.join(train_path, 'labels', '*.txt')):
		os.remove(file)

	for file in glob.glob(os.path.join(valid_path, 'images', '*.jpg')):
		os.remove(file)

	for file in glob.glob(os.path.join(valid_path, 'labels', '*.txt')):
		os.remove(file)
	
	data = bv.load_data(path=data_path)
	backgrounds = load_backgrounds(data_path)

	print(f'Data samples: {len(data)}')
	print(f'Image shape: {data[0][0].shape}')
	print(f'Label shape: {data[0][1].shape}')
	print(f'Backgrounds: {len(backgrounds)}')

	generated_samples = -1

	for image, [[c,x,y,w,h]] in data:

		images, labels = augment_image(image, c, x, y, w, h, backgrounds)
		print('Saving batch')
		generated_samples = save_generated_data(images, labels, generated_samples)

	print(f'Generated {generated_samples} images and labels')
		

if __name__ == '__main__':
	if len(sys.argv) > 1:
		main(sys.argv[1])
