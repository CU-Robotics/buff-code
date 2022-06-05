#! /usr/bin/env python3
import os
import sys
import cv2
import yaml
import glob
import shutil
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
		x = np.random.randint(0, image.shape[1] - h)
		y = np.random.randint(0, image.shape[0] - w)

		return image[y:y+h,x:x+w]

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
	filtered = cv2.GaussianBlur(cropped.copy(), [3,3], 0.8, 0)
	# hsv_image = cv2.cvtColor(filtered, cv2.COLOR_BGR2HSV)

	return filtered# hsv_image, cropped

def random_resize(image):
	r = np.random.uniform(0.5, 0.8)
	h = max(20, min(round(image.shape[0] * r), 300))
	w = max(20, min(round(image.shape[1] * r), 300))
	return cv2.resize(image, (w, h))

def mask_background(background, mask, x, y):
	mask_h, mask_w = mask.shape
	zero_mask = np.zeros(background.shape[:2], dtype=np.uint8)
	zero_mask[y: y + mask_h, x: x + mask_w] = mask

	return zero_mask

def pad_image(mask, h, w, x, y):
	mask_h, mask_w, mask_c = mask.shape
	padded_image = np.zeros((h,w,mask_c), dtype=np.uint8)
	padded_image[y: y + mask_h, x: x + mask_w, :] = mask

	return padded_image

def pad_mask(mask, h, w, x, y):
	mask_h, mask_w = mask.shape
	padded_image = np.zeros((h,w), dtype=np.uint8)
	padded_image[y: y + mask_h, x: x + mask_w] = mask

	return padded_image


def generate_images(image, c, x, y, w, h, backgrounds):

	n_samples = 5
	augmented_images = []
	augmented_labels = []

	low = np.array([0,120,0])
	high = np.array([100,255,100])

	cropped = crop_image(image, x, y, w, h)

	mask = cv2.bitwise_not(cv2.inRange(cropped, low, high))

	rgb_mask = cv2.bitwise_and(cropped.copy(), cropped.copy(), mask=mask)

	for i in range(n_samples):
		for l in range(n_samples):

			background_idx = (len(backgrounds) - 1) * np.random.rand(n_samples)

			rgb_mask = random_resize(rgb_mask.copy())
			mask_h, mask_w, mask_c = rgb_mask.shape

			mask = cv2.resize(mask, (mask_w, mask_h))

			background = backgrounds[int(background_idx[l])]
			back_h, back_w, back_c = background.shape

			augmented_images.append(background)
			augmented_labels.append([[]])

			for k in range(n_samples):
				x = np.random.randint(0, back_w - mask_w)
				y = np.random.randint(0, back_h - mask_h)

				zero_mask = mask_background(background, mask, x, y)
				inv_zero_mask = cv2.bitwise_not(zero_mask)

				padded_mask = pad_image(rgb_mask, back_h, back_w, x, y)
				
				background_mask = cv2.bitwise_and(background, background, mask=inv_zero_mask)
				augmented_image = cv2.add(background_mask, padded_mask)
				label = [[1, (x + (mask_w/2)), (y + (mask_h/2)), mask_w, mask_h]]
				
				augmented_images.append(augmented_image)
				augmented_labels.append(label)

				zero_mask = mask_background(background, mask, x, y)
				inv_zero_mask = cv2.bitwise_not(zero_mask)

				padded_mask = pad_image(cv2.cvtColor(rgb_mask, cv2.COLOR_RGB2BGR), back_h, back_w, x, y)
				
				background_mask = cv2.bitwise_and(background, background, mask=inv_zero_mask)
				augmented_image = cv2.add(background_mask, padded_mask)
				label = [[0, (x + (mask_w/2)), (y + (mask_h/2)), mask_w, mask_h]] # red is default

				augmented_images.append(augmented_image)
				augmented_labels.append(label)

				zero_mask = mask_background(background, mask, x, y)
				inv_zero_mask = cv2.bitwise_not(zero_mask)

				padded_mask = pad_image(cv2.cvtColor(rgb_mask, cv2.COLOR_RGB2BGR), back_h, back_w, x, y)
				
				background_mask = cv2.bitwise_and(background, background, mask=inv_zero_mask)
				augmented_image = cv2.add(background_mask, padded_mask)
				label = [[0, (x + (mask_w/2)), (y + (mask_h/2)), mask_w, mask_h]] # red is default

				augmented_images.append(augmented_image)
				augmented_labels.append(label)

	return None, None

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

	for i, labelf in enumerate(label_files):
		imfile = os.path.join(im_path, labelf[m:-4] + '.jpg')
		if os.path.exists(imfile):
			image = cv2.imread(imfile)
			[[c, x, y, w, h]] = bv.load_label(labelf)

			images, labels = generate_images(image, c, x, y, w, h, backgrounds)
			continue

			if generated_samples == -1:
				print(f'Data samples: {len(label_files)}')
				print(f'Image shape: {image.shape}')
				print(f'Generating {len(images)} images and labels per sample')
			
			generated_samples += len(images)

			print(f'Saving batch {i+1}/{len(label_files)}')
			bv.save_txt_label_data(images, labels, gen_path)

	print(f'Generated {generated_samples} images and labels')
	shutil.make_archive('Generated', 'zip', data_path)
		

if __name__ == '__main__':
	if len(sys.argv) > 1:
		main(sys.argv[1])
