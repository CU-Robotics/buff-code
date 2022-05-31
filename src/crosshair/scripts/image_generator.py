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

def stretch_or_cut(image, h, w):
	if image.shape[0] > h and image.shape[1] > w:
		x = np.random.randint(0, image.shape[0] - h)
		y = np.random.randint(0, image.shape[1] - w)

		return image[int(y):int(y+h),int(x):int(x+w)]

	else:
		return cv2.resize(image, (h, w))

def load_backgrounds(data_path):
	images = []
	for file in glob.glob(os.path.join(data_path, 'background', '*')):
		image = cv2.imread(file)
		images.append(stretch_or_cut(image, 320, 320))

	return images

def preprocess(image, x, y, w, h):

	cropped = crop_image(image, x, y, w, h)
	filtered = cv2.GaussianBlur(cropped.copy(), [3,3], 0.6, 0)
	hsv_image = cv2.cvtColor(filtered, cv2.COLOR_BGR2HSV)

	return hsv_image, cropped

def random_resize(image):
	r = np.random.uniform(0.35, 1.0)
	h = round(image.shape[0] * r)
	w = round(image.shape[1] * r)
	return cv2.resize(image, (h, w))

def mask_background(background, mask, x, y):
	mask_h, mask_w = mask.shape
	# print(f'masking {mask_h}, {mask_w}, {x}, {y}')
	zero_mask = np.zeros(background.shape[:2], dtype=np.uint8)
	zero_mask[round(y):round(y+mask_h),round(x):round(x+mask_w)] = mask

	return zero_mask

def pad_image(mask, h, w, x, y):
	mask_h, mask_w, mask_c = mask.shape
	padded_image = np.zeros((h,w,mask_c), dtype=np.uint8)
	padded_image[round(y):round(y+mask_h),round(x):round(x+mask_w)] = mask

	return padded_image

def generate_images(image, c, x, y, w, h, backgrounds):

	n_samples = 3
	augmented_images = []
	augmented_labels = []

	low = np.array([180,180,180])
	high = np.array([255,255,255])

	hsv_image, cropped = preprocess(image, x, y, w, h)

	mask = cv2.bitwise_not(cv2.inRange(hsv_image, low, high))
	rgb_mask = cv2.bitwise_and(cropped.copy(), cropped.copy(), mask=mask)

	for i in range(n_samples):
		background_idx = (len(backgrounds) - 1) * np.random.rand(n_samples)

		rgb_mask = random_resize(rgb_mask.copy())
		mask = cv2.resize(mask, rgb_mask.shape[:2])
		background = backgrounds[int(background_idx[i])]

		mask_h, mask_w, mask_c = rgb_mask.shape
		back_h, back_w, back_c = background.shape
		print(f'background {background.shape}')
		print(f'mask {mask.shape}')

		for k in range(n_samples):
			x = np.random.randint(0, back_h - mask_h)
			y = np.random.randint(0, back_w - mask_w)

			zero_mask = mask_background(background, mask, x, y)
			inv_zero_mask = cv2.bitwise_not(zero_mask)

			padded_mask = pad_image(rgb_mask, back_h, back_w, x, y)
			
			background_mask = cv2.bitwise_and(background, background, mask=inv_zero_mask)
			augmented_image = cv2.add(background_mask, padded_mask)
			label = [[1, x + (mask_w/2), y + (mask_h/2), mask_w, mask_h]] # red is default

			bv.display_annotated(augmented_image, label)

			augmented_images.append(augmented_image)
			augmented_labels.append(label)

			zero_mask = mask_background(background, mask, x, y)
			inv_zero_mask = cv2.bitwise_not(zero_mask)

			padded_mask = pad_image(cv2.cvtColor(rgb_mask, cv2.COLOR_RGB2BGR), back_h, back_w, x, y)
			
			background_mask = cv2.bitwise_and(background, background, mask=inv_zero_mask)
			augmented_image = cv2.add(background_mask, padded_mask)
			label = [[0, x + (mask_w/2), y + (mask_h/2), mask_w, mask_h]] # red is default

			augmented_images.append(augmented_image)
			augmented_labels.append(label)

	return augmented_images, augmented_labels



def main(data_dir):
	project_root = os.getenv('PROJECT_ROOT')

	data_path = os.path.join(project_root, 'data', data_dir)
	im_path = os.path.join(data_path, 'train', 'images')
	label_path = os.path.join(data_path, 'train', 'labels')

	gen_path = os.path.join(project_root, 'data', 'Generated')
	train_path = os.path.join(gen_path, 'train')
	valid_path = os.path.join(gen_path, 'valid')

	bv.clear_generated(gen_path)
	bv.make_generated(gen_path)
	
	backgrounds = load_backgrounds(data_path)

	print(f'Loading data from: {data_path}')
	print(f'Backgrounds: {len(backgrounds)}')

	generated_samples = -1

	m = len(label_path) + 1
	label_files = glob.glob(os.path.join(label_path, '*.txt'))

	for labelf in label_files:
		imfile = os.path.join(im_path, labelf[m:-4] + '.jpg')
		if os.path.exists(imfile):
			image = cv2.imread(imfile)
			[[c, x, y, w, h]] = bv.load_label(labelf)

			images, labels = generate_images(image, c, x, y, w, h, backgrounds)

			if generated_samples == -1:
				print(f'Data samples: {len(label_files)}')
				print(f'Image shape: {image.shape}')
				print(f'Generating {len(images)} images and labels per sample')

			print('Saving batch')
			bv.save_txt_label_data(images, labels, gen_path)

	print(f'Generated {generated_samples} images and labels')
		

if __name__ == '__main__':
	if len(sys.argv) > 1:
		main(sys.argv[1])
