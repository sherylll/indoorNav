from scipy import misc
import matplotlib.pyplot as plt
import matplotlib.colors as cls
import numpy as np
"""
doesnt work so well on testMap
"""

#import imageio

#imread returns a numpy array
im = misc.imread('testMap.png')
shape = im.shape
h = shape[0]
w = shape[1]
matrix = np.zeros(shape[:2])

#converting to matrix with only ones and zeros
for i in range(h):
	for j in range(w):
		matrix[i][j]=float.fromhex('%02x%02x%02x' % tuple(im[i][j]))

thresh = (np.amax(matrix) + np.amin(matrix))/2
for i in range(h):
	for j in range(w):
		if matrix[i][j]<thresh:
			matrix[i][j] = 1
		else: 
			matrix[i][j] = 0

#vertical scan
line_detector = 20
matrix_corr = np.zeros((h,w))
for i in range(h):
	if abs(np.sum(matrix[i])-np.sum(matrix[i-1])) > line_detector and \
	np.sum(matrix[i]) > line_detector*3:
		matrix_corr[i]=matrix[i]
for j in range(w):
	#print j
	if abs(np.sum(matrix[:,j])-np.sum(matrix[:,j-1])) > line_detector\
	and np.sum(matrix[:,j])>line_detector*2:
		matrix_corr[:,j]=matrix[:,j]
print matrix
plt.imshow(matrix_corr)
plt.show()
