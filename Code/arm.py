import numpy as np 
import math

class Manipulator:

	def __init__(self):
		# End Effector Position
		# Is position in x,y,z,thetac
		#set the default x to 12.951, position at 0,0,0,0 angle position
		self.endEffectorPos = [12.951,0,0,0]
		
		# Gripper Variable
		# zero = closed   one = open
		self.Gripper = 0

		# DH Table parameters of manipulator

		# Angle Position
		# Angles of the 4 joints
		# These are the angle variables origionally set to 0,0,0,0
		# Angles are in degrees
		self.angles = [0,0,0,0]

		# link lengths of manipulator (a values)
		# these links correspong to the a values in the
		# DH parameters, all links for the manipulator ar a lengths
		# and are initalized here
		self.lengths = [3.704,2.626,2.995,5.026]

		# alpha values
		# these correspond to the alpha values in the DH
		# For this manipulator only the first joint has one.
		# This variable is in radians
		self.alphas = [-1.57,0,0,0]

		# d values
		# these are the d lengths in the DH table
		# for this manipulator these are all zero
		self.dvalues = [0,0,0,0]

	def forwardKinematics(self,angles):
		# Calculate end effector position from angle positions
		# uses DH table values above to calculate T matrix for manipualator
		# and then calculated end effector position

		#convert to radians
		angles = [math.radians(x) for x in angles] 

		#Get transformation matrix
		H1 = self.DHtable(angles,0)
		H2 = self.DHtable(angles,1)
		H3 = self.DHtable(angles,2)
		H4 = self.DHtable(angles,3)

		#Vector to get x,y,z position from transformation matrix
		#Is set as a matrix because it needs to be a 4x1, remeber this when trying to access it
		#must access it is a matrix not an array
		d = [[0],[0],[0],[1]]

		#Calculate matricies for end effector
		T_joint1 = H1*d
		T_joint2 = H1*H2*d
		T_joint3 = H1*H2*H3*d
		T_joint4 = H1*H2*H3*H4

		#Position of end effector
		EndEffector = T_joint4*d
		T_joint4 = T_joint4*d

		#calculate orentation of end effector
		EndEffector[3] = (angles[1]+angles[2]+angles[3])*180/math.pi

		return T_joint1,T_joint2,T_joint3,T_joint4,EndEffector

	def inverseKinematics(self,endEffector):
		# Calculate angles from angle positions
		# inverseKin corresponding matlab function

		#Calculate theta values
		theta1 = math.atan2(endEffector[1],endEffector[0])

		#Create variables to simplify theta calculations
		x = math.sqrt(endEffector[0]**2 + endEffector[1]**2) - self.lengths[0]
		zc = endEffector[2]
		zc = -1*zc
		thetac=endEffector[3]*math.pi/180
		#Calculate rest of thetas
		
		#Do not take inverse cos because if costheta3 is >1 then an imaginary number will be returned
		#This means that there is no real feasible inverse kinematics solution and that the program will
		# in fact break. :O
		costheta3 = (((zc-self.lengths[3]*math.sin(thetac))**2+(x-self.lengths[3]*math.cos(thetac))**2-self.lengths[1]**2-self.lengths[2]**2)/(2*self.lengths[1]*self.lengths[2]))
		
		#If not possible to solve kinematics give up, stop calculating and return failure
		if abs(costheta3) > 1:
			inversePossible = 0
			angles =[0,0,0,0]
		else:
			#if inverse is possible finish computing it
			inversePossible = 1
			theta3 = math.acos(costheta3)
			theta2 = math.atan((zc-self.lengths[3]*math.sin(thetac))/(x-self.lengths[3]*math.cos(thetac))) -math.atan((self.lengths[2]*math.sin(theta3))/(self.lengths[1]+self.lengths[2]*math.cos(theta3)))
			theta4 = thetac-theta2-theta3
			#save angles to array
			angles = [theta1,theta2,theta3,theta4]

		#convert angles to degrees
		angles = [x * 180/math.pi for x in angles]
		return angles, inversePossible

	def DHtable(self,angles,i):
		# Calculate transformation matirix of arm at given angles
		# DH3 corresponding matlab function
		T_znm1 = np.matrix([[1, 0, 0, 0], [0,1,0,0], [0,0,1,self.dvalues[i]], [0,0,0,1]])
		R_znm1 = np.matrix([[math.cos(angles[i]),-math.sin(angles[i]),0,0], [math.sin(angles[i]),math.cos(angles[i]),0,0], [0,0,1,0], [0,0,0,1]])
		T_xn = np.matrix([[1, 0, 0, self.lengths[i]], [0,1,0,0], [0,0,1,0], [0,0,0,1]])
		R_xn = np.matrix([[1, 0, 0, 0], [0,math.cos(self.alphas[i]),-math.sin(self.alphas[i]),0], [0,math.sin(self.alphas[i]),math.cos(self.alphas[i]),0], [0,0,0,1]])
		
		#Multiply to get matrix for this joint
		T = T_znm1*R_znm1*T_xn*R_xn
		
		return T

	def calculateThetaC(self,endEffector,minAngle):
		# Calculate a thetaC for a desired X,Y,Z position, if cannot go to position return 0
		# calculateAngles corresponding matlab function, part of pathPlanning file

		canReach = False

		#Search thetac values from -30 to 30, these values 
		for thetac in xrange(minAngle,90):
			endEffector[3] = thetac 
			Angles,inversePossible = self.inverseKinematics(endEffector)

			#If angles are all real and within constraints solutions have been
			#found
			if inversePossible and self.inAngleConstriants(Angles):
				#theta c makes possible
				print thetac
				canReach = True
				break


		return Angles,canReach,thetac	

	def inAngleConstriants(self,angles):
		# Return 1 if angle values are within angle constraints, 0 if not
		# isInAngleConstraints is corresponding matlab function, part of pathPlanning file

		# isInAngleConstraints = 1
		return ((angles[0]<=90) & (angles[0]>=-90) & (angles[1]>=-50) & (angles[1]<=120) & (angles[2]>=-55) & (angles[2]<=120) & (angles[3]>=-110) & (angles[3]<=90))
