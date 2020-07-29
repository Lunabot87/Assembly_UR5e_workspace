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

	target_radius = 0.2
	target_center = marker5 # [x, y]
	# target_center = marker1 # [x, y]
	# target_center = (marker4*9+marker5)/10 # [x, y]

	total_hold_points = []
	for i in range(len(parts)):
		is_online = check_online(parts[i], target_center)
		if is_online:
			hold_points_on = calculate_hold_point_on_line(parts[i], target_center, target_radius)
			if hold_points_on is not None:
				for j in range(len(hold_points_on)):
					total_hold_points.append(hold_points_on[j])
		else:
			hold_points_off = calculate_hold_point_off_line(parts[i], target_center, target_radius)
			if hold_points_off is not None:
				for j in range(len(hold_points_off)):
					total_hold_points.append(hold_points_off[j])

	for i in range(len(parts)):
		part = parts[i]
		plt.plot([part[0][0], part[1][0]], [part[0][1], part[1][1]], parts_color[i]+'o-')
	plt.plot(target_center[0], target_center[1], "c*", markersize=10)
	
	for j in range(len(total_hold_points)):
		dist_from_target_center = np.linalg.norm(total_hold_points[j] - target_center)
		print "x={}, y={}, dist={}".format(total_hold_points[j][0], total_hold_points[j][1], dist_from_target_center)
		plt.plot(total_hold_points[j][0], total_hold_points[j][1], "m*", markersize=10)
	
	plt.show()	

def check_online(X, R, online_thresh=0.0000001):
	(x1, y1) = (X[0][0], X[0][1])
	(x2, y2) = (X[1][0], X[1][1])
	(xr, yr) = (R[0], R[1])

	# get linear equation from two points X1, X2
	m = (y2 - y1)/(x2 - x1)
	n = y1 - m*x1

	# check if point R is not on line X12
	if (m*xr+n - yr) < online_thresh:
		return True
	else:
		return False

def calculate_hold_point_on_line(X, R, r):
	(x1, y1) = (X[0][0], X[0][1])
	(x2, y2) = (X[1][0], X[1][1])
	(xr, yr) = (R[0], R[1])
	
	# get vector of line(x)
	X12 = X[1]-X[0]
	x = X12/np.linalg.norm(X12)

	# get two points that are located at r distance from R
	Q1 = R + r*x
	Q2 = R - r*x

	# check if two points are on line X12
	CrossPts = []
	minx = min(x1, x2)
	maxx = max(x1, x2)
	# print miny, maxy
	if (minx <= Q1[0] <= maxx):
		CrossPts.append(Q1)
	if (minx <= Q2[0] <= maxx):
		CrossPts.append(Q2) 
	
	if len(CrossPts) > 0:
		return CrossPts
	else:
		return None

def calculate_hold_point_off_line(X, R, r, on_line_thresh=0.0000001):
	(x1, y1) = (X[0][0], X[0][1])
	(x2, y2) = (X[1][0], X[1][1])
	(xr, yr) = (R[0], R[1])

	# get P
	den = ((y2-y1)/(x2-x1) + (x2-x1)/(y2-y1))
	num = ((y2-y1)/(x2-x1)*x1 +(x2-x1)/(y2-y1)*xr + yr - y1)
	xp = num/den
	yp = -(x2-x1)/(y2-y1)*(xp-xr) + yr
	P = np.array([xp, yp])
	
	# get d
	RP = P - R
	d = np.linalg.norm(RP)
	
	# get vector of line(x)
	X12 = X[1]-X[0]
	x = X12/np.linalg.norm(X12)
	minx = min(x1, x2)
	maxx = max(x1, x2)
	
	# get Q
	CrossPts = []
	if d>0 and d<r:
		l = m.sqrt(r*r - d*d)
		Q1 = P + l*x
		Q2 = P - l*x
		if (minx <= Q1[0] <= maxx):
			CrossPts.append(Q1)
		if (minx <= Q2[0] <= maxx):
			CrossPts.append(Q2)

		if len(CrossPts) > 0:
			return CrossPts
		else:
			return None
	else:
		return None

if __name__ == '__main__':
    main()