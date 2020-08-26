#!/usr/bin/env python
import rospy
import rospkg
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32
from std_msgs.msg import String

import numpy as np
import pandas as pd
import math as m
import os
import csv

from pprint import pprint


class XlsxReader():
	def __init__(self, xlsxPath):
		self.xlsxPath = xlsxPath
		self.xlsx = pd.read_excel(self.xlsxPath)

class readCSV():
	def __init__(self, csvPath):
		self.csvPath = csvPath
		self.csvData = open(self.csvPath)
		self.csvReader = csv.reader(self.csvData)
		self.csv = list(self.csvReader)

	def makeHoleDict(self):
		partDict = {}
		holeInfo = {"hole_name": String(), "hole_pose": Pose(), "hole_size": Float32()}
		partInfo = {"part_name": String(), "part_vol": Vector3()}
		
		for line in self.csv[2:]:
			print(line)
			


def main():
	r = rospkg.RosPack()
	pkgPath = r.get_path("object_description")
	configPath = os.path.join(pkgPath, "config")
	xlsxPath = os.path.join(configPath, "parts_info.xlsx")
	csvPath = os.path.join(configPath, "partsInfo.csv")
	
	xr = XlsxReader(xlsxPath)
	rc = readCSV(csvPath)

	rc.makeHoleDict()


if __name__ == "__main__":	
	try :
		main()
	except rospy.ROSInterruptException:
		pass