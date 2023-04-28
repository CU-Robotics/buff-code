#!/usr/bin/env python3
"""
	Project: trainer (Crosshair)
	Author: Mitchell D Scott
	Description:
	  A script to continuously train and deploy models
"""

import os
import sys
import yaml
import glob
import shutil
import argparse
from PIL import Image
import subprocess as sb

def get_current_exp(output_dir):
    exp = ''
    for file_path in glob.glob(os.path.join(output_dir, 'exp*')):
        file_name = file_path.split('/')[-1]
        if len(file_name) > 3:
            n = int(file_name[3:])
            if exp == '' or n > int(exp):
                exp = f'{n}'
            
    return exp

def write_data_yaml(data_path):
	data_file = os.path.join(data_path, 'data.yaml')
	train_path = os.path.join(data_path, 'train')
	valid_path = os.path.join(data_path, 'valid')
	test_path = os.path.join(data_path, 'test')

	with open(data_file, 'w+') as f:
		if os.path.exists(train_path):
			f.write(f'train: {train_path}\n')

		if os.path.exists(valid_path):
			f.write(f'val: {valid_path}\n')

		if os.path.exists(test_path):
			f.write(f'test: {test_path}\n')

		f.write(f'\nnc: 3\nnames: [\'blue-armor\', \'red-armor\', \'robot\']')

def clear_dataset(dataset):
	shutil.rmtree(os.path.join(dataset, 'train'))
	shutil.rmtree(os.path.join(dataset, 'valid'))
	shutil.rmtree(os.path.join(dataset, 'test'))
	os.remove(os.path.join(dataset, 'data.yaml'))

def unzip_dataset(data_dir, datazip):
	dataset = datazip[:-4]
	print(f'unziping {os.path.join(data_dir, dataset)}')
	shutil.unpack_archive(os.path.join(data_dir, datazip), data_dir)
	# unpack_archive has weird behavior probably from the way files were zipped (roboflow zips content not dir)
	if not os.path.exists(os.path.join(data_dir, dataset)):
		clear_dataset(data_dir)
		shutil.unpack_archive(os.path.join(data_dir, datazip), os.path.join(data_dir, dataset))

	data_path = os.path.join(data_dir, dataset)
	write_data_yaml(data_path)

	return data_path

def zip_dataset(dataset_path):
	print(f'ziping {dataset_path}.zip')
	write_data_yaml(dataset_path)

	if os.file.exists(os.path.join(dataset_path, 'README.dataset.txt')):
		os.remove(os.path.join(dataset_path, 'README.dataset.txt'))
	if os.file.exists(os.path.join(dataset_path, 'README.roboflow.txt')):
		os.remove(os.path.join(dataset_path, 'README.roboflow.txt'))

	shutil.make_archive(dataset_path, 'zip', dataset_path)

def resize_dataset(datadir, w, h):
	rezip = False
	for filename in glob.iglob(datadir + '**/**/*.jpg', recursive=True):
		im = Image.open(filename)
		if not im.size == (h,w):
			imResize = im.resize((h,w), Image.ANTIALIAS)
			imResize.save(filename , 'JPEG', quality=90)
			rezip = True

	if rezip:
		zip_dataset(datadir)

def train_model(model):

	runs = 0
	project_root = os.getenv('PROJECT_ROOT')

	model_dir = os.path.join(project_root, 'buffpy', 'data', 'models')
	yolo_dir = os.path.join(project_root, '..', 'yolov5')
	data_dir = os.path.join(project_root, 'data')
	output_dir = os.path.join(data_dir, 'runs')

	if os.path.exists(output_dir):
		shutil.rmtree(output_dir)

	os.mkdir(output_dir)

	exp = get_current_exp(output_dir)
	model_path = os.path.join(model_dir, model)

	default_args = ['--img', '640','--batch', '32', '--epochs', '50', '--project', output_dir]

	if not os.path.exists(data_dir):
		print('No training Data')
		return

	while 1:
		data_zips = glob.glob(os.path.join(data_dir, '*.zip'))
		if len(data_zips) == 0:
			print('No training Data')
			return

		for data_zip in data_zips:
			args = []

			data_path = unzip_dataset(data_dir, data_zip)
			data_file = os.path.join(data_path, 'data.yaml')

			if not os.path.exists(data_file):
				print(f'Dataset {data_file} data.yaml does not exist')
				continue

			if runs == 0:
				resize_dataset(data_path, 640, 640)

			if not os.path.exists(model_path):
				# args += ['--weights', '""']
				args += ['--cfg', 'yolov5s.yaml']
			
			else:
				args += ['--weights', model_path]

			args += ['--data', data_file]

			cmd = ['python3', os.path.join(yolo_dir, 'train.py')] + default_args + args
			print(f'Executing {cmd}')
			sb.run(cmd)

			shutil.copy(os.path.join(output_dir, 'exp' + exp, 'weights', 'best.pt'), os.path.join(model_dir, f'{model}.pt'))
			shutil.rmtree(data_path)

			cmd = ['python3', os.path.join(yolo_dir, 'export.py'), '--rknpu', 'RK3588', '--weights', os.path.join(model_dir, f'{model}.pt'),  '--include', 'onnx']
			sb.run(cmd)

			exp = get_current_exp(output_dir)

			model_path = os.path.join(model_dir, f'{model}.pt')


if __name__ == '__main__':
	if len(sys.argv) > 1:
		train_model(sys.argv[1])
	else:
		train_model("buffnet")