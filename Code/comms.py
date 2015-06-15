import struct
import numpy as np
import subprocess
""" This is for communicating with the computer to get 
data from the camera. Use socket
"""
class GetObsticles:

	def __init__(self):
		#Distance between x axis of camera and arm measured along z axis
		self.distz = 29.8
		#Distance between Z axis of camera and arm measured along x axis
		self.distx = 6.7
		#Distance between y axis of camera and arm measured along y axis
		self.disty = 0.2

	def getData(self):

		#Run tracking program to locate first obsticle
		print "About to Run Program"
		proc = subprocess.Popen(["./trackend", "1"],bufsize = 100, stdout=subprocess.PIPE)
		print "Running Program"
		for x in range(0, 10):
			proc.stdout.readline()
		x = y = z = 0.0001 
		count = 0
		for i in range(0, 10):
			obsticleString = proc.stdout.readline()
			#print "Python:",obsticleString 
			obsticleList = obsticleString.split(' ')
			x = x + float(obsticleList[0])
			y = y + float(obsticleList[1])
			z = z + float(obsticleList[2])
			radius = float(obsticleList[3])
			count = count+1
			#print x/count,y/count,z/count
		x = x/10
		y = y/10
		z = z/10
		Obsticle1 =  self.convertObsticles(x,y,z,radius)
		print Obsticle1
		proc.terminate()

		#Run tracking program to locate second obsticle
		proc = subprocess.Popen(["./trackend", "2"],stdout=subprocess.PIPE)
		print "Running Program"
		for x in range(0, 10):
			proc.stdout.readline()
		x = y = z = count = 0
		for i in range(0, 10):
			obsticleString = proc.stdout.readline()
			#print "Python:",obsticleString 
			obsticleList = obsticleString .split(' ')
			x = x + float(obsticleList[0])
			y = y + float(obsticleList[1])
			z = z + float(obsticleList[2])
			radius = float(obsticleList[3])
			count = count+1
			#print x/count,y/count,z/count
		x = x/10
		y = y/10
		z = z/10
		Obsticle2 = self.convertObsticles(x,y,z,radius)
		proc.terminate()

		#Run tracking program to locate third obsticle
		proc = subprocess.Popen(["./trackend", "3"],stdout=subprocess.PIPE)
		print "Running Program"
		for x in range(0, 10):	
			proc.stdout.readline()
		x = y = z = count = 0
		for i in range(0, 10):
			obsticleString = proc.stdout.readline()
			#print "Python:",obsticleString 
			obsticleList = obsticleString.split(' ')
			x = x + float(obsticleList[0])
			y = y + float(obsticleList[1])
			z = z + float(obsticleList[2])
			radius = float(obsticleList[3])
			count = count+1
			#print x/count,y/count,z/count
		x = x/10
		y = y/10
		z = z/10
		Obsticle3 = self.convertObsticles(x,y,z,radius)
		Obsticle3[2] = Obsticle3[2]+3
		proc.terminate()
		return Obsticle1,Obsticle2,Obsticle3

	def convertObsticles(self,x,y,z,r):
		# Function to get camera x,y,z coordinates into arm xyz coordinates
		# If no object has been found, radius of camera tracking is 0 return -1,-1,-1
		# as the camera coordintes of the object
		Obsticle = [-1,-1,-1,0]
		if (r!=0):
			Obsticle[0] = y + self.distx 
			Obsticle[1] = -1*x - self.disty
			Obsticle[2] = self.distz - z
		return Obsticle
