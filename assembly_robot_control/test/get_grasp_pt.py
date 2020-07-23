import math as m
import numpy as np
import matplotlib.pylab as plt

def main():
	marker1 = [0.2508, -0.0606] # [[x1, y1], [x2, y2]]
	marker2 = [0.2385, 0.3361]
	marker3 = [-0.17189, 0.3264]
	marker4 = [-0.19571, -0.5452]
	marker5 = [-0.12171, -0.0567]

	ref = [0.0, 0.0] #-np.array(marker5)
	marker1 = np.array(marker1) + ref
	marker2 = np.array(marker2) + ref
	marker3 = np.array(marker3) + ref
	marker4 = np.array(marker4) + ref
	marker5 = np.array(marker5) + ref

	part1 = np.array([marker1, marker2])
	part2 = np.array([marker1, marker5])
	part3 = np.array([marker3, marker5])
	part4 = np.array([marker4, marker5])
	parts = np.array([part1, part2, part3, part4])
	parts_color = ['r', 'g', 'b', 'k']

	target_radius = 0.05
	target_center = (marker5*2+marker4)/3 # [x, y]

	hold_points = []
	for i in range(len(parts)):
		hold_point = calculate_hold_point_on_line(parts[i], target_center, target_radius)
		if hold_point is not None:
			hold_points.append(hold_point)
	# hold_point, hold_point1 = calculate_hold_point(parts[1], target_center, target_radius)

	for i in range(len(parts)):
		part = parts[i]
		plt.plot([part[0][0], part[1][0]], [part[0][1], part[1][1]], parts_color[i]+'o-')
	plt.plot(target_center[0], target_center[1], "*")
	
	for j in range(len(hold_points)):
		print hold_points[j][0], hold_points[j][1]
		plt.plot(hold_points[j][0], hold_points[j][1], "r*")

	
	# print hold_point1
	# 	plt.plot(hold_point1[0], hold_point1[1], "k*")

	plt.show()	

def calculate_hold_point_on_line(X, R, r, on_line_thresh=0.0000001):
	(x1, y1) = (X[0][0], X[0][1])
	(x2, y2) = (X[1][0], X[1][1])
	(r1, r2) = (R[0], R[1])

	# get linear equation from two points X1, X2
	m = (y2 - y1)/(x2 - x1)
	n = x1 - m*x1

	# check if point R is not on line X12
	if (m*r1+n - r2) > on_line_thresh:
		return None
	
	# get vector of line
	X12 = X[1]-X[0]
	x = X12/np.linalg.norm(X12)

	# get two points that are located at r distance from R
	Q1 = R + r*x
	Q2 = R - r*x
	print "="*30
	print X[0], X[1]
	print "X12: " + str(X12)
	print "x: " + str(x)
	print "R: "+str(R)
	print Q1, Q2


	# check if two points are on line X12
	miny = min(y1, y2)
	maxy = max(y1, y2)
	print miny, maxy
	if (miny < (m*Q1[0]+n) < maxy):
		return Q1
	elif (miny < (m*Q2[0]+n) < maxy):
		return Q2
	else:
		return None





def calculate_hold_point_off_line(X, R, r):
	# get P
	(x1, y1) = (X[0][0], X[0][1])
	(x2, y2) = (X[1][0], X[1][1])
	(r1, r2) = (R[0], R[1])

	den = ((y2-y1)/(x2-x1) + (x2-x1)/(y2-y1))
	num = ((y2-y1)/(x2-x1)*x1 +(x2-x1)/(y2-y1)*r1 +r2)
	xp = num/den
	yp = -(x2-x1)/(y2-y1)*(xp-r1) + r2
	P = np.array([xp, yp])
	
	# get a_(vector)
	X12 = X[1]-X[0]
	a_ = X12/np.linalg.norm(X12)

	# get d
	RP = P - R
	d = np.linalg.norm(RP)

	print d
	if d>0 and d<r:
		l = m.sqrt(r*r - d*d)
		Q = X[0] + (P-X[0]) + l*a_
		return P, Q
	else:
		return None, None
	# return [[x1, y1], [x2, y2], ..]

def test():
	hold_pose = calculate_hold_pose(parts, hold_orients, hole, hold_boundary)


if __name__ == '__main__':
    main()