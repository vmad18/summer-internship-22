import numpy as np 
from enum import Enum


camera_mat = np.array([
		                [ 824.41491208, 0., 315.73103864 ],
		                [ 0., 825.71672105, 265.08665437 ],
		                [ 0., 0., 1. ]])

camera_mati = np.linalg.inv(camera_mat)

camera_height = 25.4 * 4.5

camera_bot = 25.4 * 2.1 * .65

translation_vect = np.asarray([0, camera_height, 0])



class Mode(Enum):

	STABLE = 1
	TEL = 2
	SEARCH = 3
	OWN = 4
	BLOB = 5