import matplotlib.pyplot as plt
import numpy as np

def drawLine(p1, p2, matrix):
	#(x1, y1) = p1
	#(x2, y2) = p2
	(y1,x1) = p1
	(y2,x2) = p2
	if x1 != x2:
		delta_y = int((y2-y1)/(x2-x1))
		for i in range(0, x2-x1+1, 1 if x2>x1 else -1):		
			matrix[x1+i][y1+delta_y*i] = 1
	else:
		for j in range(y1,y2+1,1 if y2>y1 else -1):
			#print j
			matrix[x1][j] = 1
	
N = 100
#M = 100
m = np.zeros((N+1,N+1))

drawLine((0,0),(100,0),m)	
drawLine((0,0),(0,100),m)
drawLine((100,0),(100,100),m)
drawLine((0,N),(N,N),m)



	
plt.imshow(m[::-1])
plt.show()
