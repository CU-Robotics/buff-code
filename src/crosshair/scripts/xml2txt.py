#! /usr/bin/env python3
import os
import cv2
import sys
import glob
import numpy as np
import buffvision as bv
import xml.etree.ElementTree as ET

def get_txt_from_xml(label, image_shape):
	labels = []

	objects = label.findall('object')
	for obj in objects:
		name = obj.find('name').text
		if name == 'armor':
			color = obj.find('armor_color')
			if not color is None:
				if color.text == 'blue':
					cl = 0

				elif color.text == 'red':
					cl = 1

				elif color.text == 'purple':
					cl = 2

				elif color.text == 'grey':
					continue

		elif not name == 'base':
			cl = 3

		else:
			continue

		bound = obj.find('bndbox')
		xmin = round(float(bound.find('xmin').text))
		ymin = round(float(bound.find('ymin').text))
		xmax = round(float(bound.find('xmax').text))
		ymax = round(float(bound.find('ymax').text))
		xcenter = (xmax + xmin) / (2 * image_shape[1])
		ycenter = (ymax + ymin) / (2 * image_shape[0])
		width = abs(xmax - xmin) / image_shape[1]
		hieght = abs(ymax - ymin) / image_shape[0]

		labels.append([cl, xcenter, ycenter, width, hieght])

	return labels

def fix_collision(x, y, w, h, labels):

	adjusted = False
	x1,y1,x2,y2 = bv.xywh2xyxy(x, y, w, h)


	for [c, xl, yl, wl, hl] in labels:
		xa,ya,xb,yb = bv.xywh2xyxy(xl, yl, wl, hl)

		if (y1 > ya and y1 < yb) or (y2 > ya and y2 < yb):
			if x1 > xa and x1 < xb:
				adjusted = True
				if x1 > (xa + xb) / 2:
					x1 = xa
				else:
					x1 = xb

			if x2 > xa and x2 < xb:
				adjusted = True
				if x2 > (xa + xb) / 2:
					x2 = xa
				else:
					x2 = xb

		if (x1 > xa and x1 < xb) or (x2 > xa and x2 < xb):
			if y1 > ya and y1 < yb:
				adjusted = True
				if y1 > (ya + yb) / 2:
					y1 = ya
				else:
					y1 = yb

			if y2 > ya and y2 < yb:
				adjusted = True
				if y2 > (ya + yb) / 2:
					y2 = ya
				else:
					y2 = yb

	return (x1 + x2) / 2, (y1 + y2) / 2, abs(x1 - x2), abs(y1 - y2), adjusted


def fit_crop(image, x, y, w, h, labels):

	max_i = 50
	for j in range(max_i):
		print(f'Fitting {j}/{max_i}')
		x, y, w, h, adj = fix_collision(x, y, w, h, labels)

		if not adj:
			return x, y, w, h

	return -1, -1, -1, -1

def filter_data(image, orig_labels):

	n_samples = 1
	img_shape = [416, 416]

	img_shape[0] = min(image.shape[0], img_shape[0])
	img_shape[1] = min(image.shape[1], img_shape[1])

	for i in range(n_samples):
		# samples between img_shape / 2 and image.shape - img_shape / 2
		w = img_shape[1] / image.shape[1]
		h = img_shape[0] / image.shape[0]
		x = np.random.uniform(w / 2, 1 - (w / 2))
		y = np.random.uniform(h / 2, 1 - (h / 2))

		x, y, w, h = fit_crop(image, x, y, w, h, orig_labels)

		if x < 0 or y < 0:
			return None, None

		labels = []
		# print(f'X range {x - w2}, {x + w2}')
		# print(f'Y range {y - h2}, {y + h2}')
		# print(f'{w2} {h2}')
		for c, xl, yl, wl, hl in orig_labels:
			if abs(xl - x) < w / 2:
				if abs(yl - y) < h / 2:
					#x1, y1, x2, y2 = bv.xywh2xyxy(xl,yl,wl,hl)
					print([c, round((xl - x + (w / 2)) * image.shape[1]), round((yl - y + (h / 2)) * image.shape[0]), wl * image.shape[1], hl * image.shape[0]])
					label = np.array([c, round((xl - x + (w / 2)) * image.shape[1]) / img_shape[1], round((yl - y + (h / 2)) * image.shape[0]) / img_shape[0], round(wl * image.shape[1]) / img_shape[1], round(hl * image.shape[0]) / img_shape[0]])
					labels.append(label)

		cropped = bv.crop_image(image.copy(), x, y, w, h)
		print(cropped.shape)
		print(img_shape)
		bv.display_annotated(cropped, labels)

	return None, None


def main(data_dir):

	project_root = os.getenv('PROJECT_ROOT')
	data_path = os.path.join(project_root, 'data', data_dir)

	gen_path = os.path.join(project_root, 'data', 'Generic')
	train_path = os.path.join(gen_path, 'train')
	valid_path = os.path.join(gen_path, 'valid')

	bv.clear_generated(gen_path)
	bv.make_generated(gen_path)

	xml_filenames = glob.glob(os.path.join(data_path, 'image_annotation', '*.xml'))

	sample_counter = 0

	print(f'{len(xml_filenames)} Samples found')
		
	for file in xml_filenames:
		xml = ET.parse(file)
		fname = file.split('/')[-1].split('.')[0]
		jpg_filename = os.path.join(data_path, 'image', fname + '.jpg')
		if os.path.exists(jpg_filename):
			image = cv2.imread(jpg_filename)
			labels = get_txt_from_xml(xml, image.shape)
			images, labels = filter_data(image, labels)
			continue
			bv.save_txt_label_data(images, labels, gen_path)
			sample_counter += 1
		else:
			print(f'No image file {jpg_filename}')

	print(f'{sample_counter} Samples converted')


if __name__ == '__main__':
	if len(sys.argv) > 1:
		main(sys.argv[1])








	