import numpy as np

def kernel_get(varx, vary, width, height, black_lane):
	if width % 2 != 1:
		raise Exception
	if height % 2 != 1:
		raise Exception
		
	
	x = np.linspace(-(width // 2), (width //2), width)
	y = np.linspace(-(height // 2), (height // 2), height) # create data points ranging evenly from before the center to after
	
	if black_lane:
		x = -no.exp(-(x**2) / (2*varx)) * ( 1 - x*x/(varx)) / varx
	else:
		x = np.exp(-(x**2) / (2*varx)) * ( 1 - x*x/(varx)) / varx
		
	y = np.exp(-1*(y*y) / (2.0*vary))
	x = np.reshape(x, (1, width))
	y = np.reshape(y, (height, 1))
	im = np.dot(y, x)
	return im