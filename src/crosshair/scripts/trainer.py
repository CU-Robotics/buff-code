#! /usr/bin/env python3
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
import subprocess

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
	os.remove(os.path.join(dataset, 'README.dataset.txt'))
	os.remove(os.path.join(dataset, 'README.roboflow.txt'))

def unzip_dataset(data_dir, datazip):
	dataset = datazip[:-4]
	shutil.unpack_archive(os.path.join(data_dir, datazip), data_dir)
	# unpack_archive has weird behavior probably from the way files were zipped (roboflow zips content not dir)
	if not os.path.exists(os.path.join(data_dir, dataset)):
		clear_dataset(data_dir)
		shutil.unpack_archive(os.path.join(data_dir, datazip), os.path.join(data_dir, dataset))

	data_path = os.path.join(data_dir, dataset)
	write_data_yaml(data_path)

	return data_path

def train_model(model):

	project_root = os.getenv('PROJECT_ROOT')
	setup_script = os.path.join(project_root, 'buffpy', 'scripts', 'yolov5_setup.bash')

	model_dir = os.path.join(project_root, 'buffpy', 'models')
	yolo_dir = os.path.join(project_root, '..', 'yolov5')
	data_dir = os.path.join(project_root, 'data')
	output_dir = os.path.join(model_dir, 'runs')

	if not os.path.exists(output_dir):
		os.mkdir(output_dir)

	exp = get_current_exp(output_dir)
	model_path = os.path.join(model_dir, model)

	subprocess.run([setup_script])

	default_args = ['--img', '320','--batch', '16', '--epochs', '50', '--cache', '--project', output_dir]

	while 1:
		data_zips = glob.glob(os.path.join(data_dir, '*.zip'))
		if len(data_zips) == 0:
			return

		if not os.path.exists(data_dir):
			print('No training Data')
			return

		for data_zip in data_zips:
			args = []

			data_path = unzip_dataset(data_dir, data_zip)
			data_file = os.path.join(data_path, 'data.yaml')

			if not os.path.exists(data_file):
				print(f'Dataset {data_file} data.yaml does not exist')
				continue

			if not os.path.exists(model_path):
				args += ['--weights', '""']
				args += ['--cfg', 'yolov5m.yaml']
			
			else:
				args += ['--weights', model_path]

			args += ['--data', data_file]

			cmd = ['python3', os.path.join(yolo_dir, 'train.py')] + default_args + args
			print(f'Executing {cmd}')
			subprocess.run(cmd)

			shutil.copy(os.path.join(output_dir, 'exp' + exp, 'weights', 'best.pt'), os.path.join(model_dir, 'buffnet.pt'))
			shutil.rmtree(data_path)

			exp = get_current_exp(output_dir)

			model_path = os.path.join(model_dir, 'buffnet.pt')


if __name__ == '__main__':
	if len(sys.argv) > 1:
		train_model(sys.argv[1])
	else:
		train_model("new")