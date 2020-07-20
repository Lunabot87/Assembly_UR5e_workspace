import math as m
import numpy as np
import matplotlib.pylab as plt

def main():
	marker1 = [0.2508, -0.0606] # [[x1, y1], [x2, y2]]
	marker2 = [0.2385, 0.3361]
	marker3 = [-0.17189, 0.3264]
	marker4 = [-0.19571, -0.5452]
	marker5 = [-0.12171, -0.0567]

	ref = -np.array(marker5)
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

	target_radius = 0.4
	target_center = (marker5*2+marker4)/3 # [x, y]

	hold_point, hold_point1 = calculate_hold_point(parts[1], target_center, target_radius)

	for i in range(len(parts)):
		part = parts[i]
		plt.plot([part[0][0], part[1][0]], [part[0][1], part[1][1]], parts_color[i]+'o-')
	plt.plot(target_center[0], target_center[1], "*")
	
	print hold_point
	print hold_point1
	if hold_point is not None:
		plt.plot(hold_point[0], hold_point[1], "r*")
		plt.plot(hold_point1[0], hold_point1[1], "k*")

	plt.show()	

def calculate_hold_point(X, R, r):
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