import os
import sys
import yaml
from pydrive.auth import GoogleAuth
from pydrive.drive import GoogleDrive

## this script needs major work 
## for now we will upload data manually

class GD_Handler:

	def __init__(self, config='config/lib/gdrive.yaml'):
		self.config = os.path.join(os.getenv('PROJECT_ROOT'), config)

		with open(self.config, 'r') as f:
			self.handle = yaml.safe_load(f)

		GoogleAuth.DEFAULT_SETTINGS['client_config_file'] = os.path.join(os.getenv('PROJECT_ROOT'), self.handle['SECRETS'])

		self.drive = self.authenticate()

	def authenticate(self):
		"""
			this function will authenticate the users google account
		"""
		gauth = GoogleAuth()
		# Creates local webserver and auto
		# handles authentication.
		if 'edge' in os.getenv('HOSTNAME'):
			gauth.CommandLineAuth()   # use cmdline on jetson in case of headless session
		else:
			gauth.LocalWebserverAuth()
		drive = GoogleDrive(gauth)
		return drive

	def uploadFolder(self, batch=None, folder='data'):

		path = os.path.join(os.getenv('PROJECT_ROOT'), folder)

		if not os.path.exists(path):
			print(f'{folder} not found: skipping...')
			return

		if batch is None:
			batch = self.handle['UNLABELED_DATA_FOLDER_ID']
		else:
			batch = self.handle[batch]
		# iterating thought all the files/folder
		# of the desired directory
		for file in os.listdir(path):
		   
			print(f'GDrive handler uploading {file}...')
			f = self.drive.CreateFile({
				'title': file,
				'parents': [{
					'kind': 'drive#fileLink',
					'teamDriveId': self.handle['TEAM_DRIVE_ID'],
					'id': batch # folder_id of the data set folder
				}]
			})
			f.SetContentFile(os.path.join(path, file))
			f.Upload()
		  
			# Due to a known bug in pydrive if we 
			# don't empty the variable used to
			# upload the files to Google Drive the
			# file stays open in memory and causes a
			# memory leak, therefore preventing its 
			# deletion
			f = None

	def uploadVideo(self, batch=None):
		"""
			Upload all of the videos in the video folder
		"""
		uploadFolder(batch, 'video')

	def uploadImages(self, batch=None):
		"""
			Upload all of the images in the image folder
		"""
		uploadFolder(batch, 'images')

	def downloadBatch(self, batch=None):
		"""
			Use this function to pull data from GDrive
		"""
		if batch is None:
			batch = self.handle['UNLABELED_DATA_FOLDER_ID']
		else:
			batch = self.handle[batch]
		
		file_list = self.drive.ListFile({'q': f'\'{batch}\' in parents and trashed=false'}).GetList()
		print(f'GDrive handler Downloading from {batch}')

		for file in file_list:
			print(f'GDrive handler Downloading {file["title"]}...')
			#data = drive.CreateFile({'id': file['id']})
			file.GetContentFile(os.path.join(os.getenv('PROJECT_ROOT'), 'data', file['title']))
			file = None

