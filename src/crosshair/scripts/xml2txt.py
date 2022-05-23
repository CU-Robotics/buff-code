#! /usr/bin/env python3
import os
import cv2
import sys
import glob
import numpy as np
import buffvision as bv
import xml.etree.ElementTree as ET

def get_txt_from_xml(label):
	labels = []

	objects = label.findall('object')
	for obj in objects:
		name = obj.find('name').text
		if name == 'armor':
			color = obj.find('armor_color')
			if not color is None:
				if color.text == 'blue':
					cl = 0

				else:
					cl = 1

		elif not name == 'base':
			cl = 2

		else:
			continue

		bound = obj.find('bndbox')
		xmin = round(float(bound.find('xmin').text))
		ymin = round(float(bound.find('ymin').text))
		xmax = round(float(bound.find('xmax').text))
		ymax = round(float(bound.find('ymax').text))
		xcenter = (xmax + xmin) / 2
		ycenter = (ymax + ymin) / 2
		width = abs(xmax - xmin)
		hieght = abs(ymax - ymin)
		labels.append([cl, xcenter, ycenter, width, hieght])

	return labels


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
			label = get_txt_from_xml(xml)
			bv.save_txt_label_data([image], [label], gen_path)
			sample_counter += 1
		else:
			print(f'No image file {jpg_filename}')

	print(f'{sample_counter} Samples converted')


if __name__ == '__main__':
	if len(sys.argv) > 1:
		main(sys.argv[1])








	