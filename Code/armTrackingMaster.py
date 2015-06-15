# This is the main function for handling path planning. It tells the camera to get data from the camera about the different
# obstacles. Asks the user what obstacle they want to send the arm to. Calculates path to that obstacle. Sends the path to the
#arm over serial communication.

import numpy as np
import arm
from PathPlanner import *
import communication
import sys
import subprocess
import comms
from tabulate import tabulate
import time

#Set arm to initial position
downlinkObject = communication.DownLink()
downlinkObject.sendValues(90,-50,-50,-50)

# First step is to get Obstacle values from the camera. This is done by using pipes to start get data and end 
# the tracking program, which is an executable called trackend, short for track end effector. This program is written in
# c++ and compiled in cmake. These files are also in this folder.

# Create object to get obstacles 
obstcleGetter = comms.GetObsticles()
# Get data of all obstacles from camera.
Obsticle1,Obsticle2,Obsticle3=obstcleGetter.getData() 
# Obsticle1 = [11,-4,-4.5,0]
# Obsticle2 = [5,11,-4.5,0]
# Obsticle3 = [0,0,0,0]

# Next step is to select which obstacle is the goal and find the corresponding angle values values for 
# this location. If it is not possible to reach this location then the program will end.

#Default no objects in workspace
inWorkspace = [0,0,0]

#Create arm object
armObject = arm.Manipulator()

#Format Points so they are pretty, thats really what most of this code is. Generating 
#a pretty table, it is pretty. I promise. It was worth the effort.
ob1 = str(Obsticle1[0])+","+str(Obsticle1[1])+","+str(Obsticle1[2])
ob2 = str(Obsticle2[0])+","+str(Obsticle2[1])+","+str(Obsticle2[2])
ob3 = str(Obsticle3[0])+","+str(Obsticle3[1])+","+str(Obsticle3[2])

#Calculate if the points are in the Workspace
Obsticle1Angles,inWorkspace[0],Obsticle1[3] = armObject.calculateThetaC(Obsticle1,-90)
Obsticle2Angles,inWorkspace[1],Obsticle2[3] = armObject.calculateThetaC(Obsticle2,-90)
Obsticle3Angles,inWorkspace[2],Obsticle3[3] = armObject.calculateThetaC(Obsticle3,-90)

# More table formatting, decide what to print in each table column 
InWorkspace = ["Not In Workspace","Not In Workspace","Not In Workspace"]
for x in range(0, 3):
	if inWorkspace[x]:
		InWorkspace[x] = "In Workspace"
FoundObject = [ob1,ob2,ob3]
if Obsticle1[0] == -1:
	FoundObject[0] = "Not Found"
if Obsticle2[0] == -1:
	FoundObject[1] = "Not Found"
if Obsticle3[0] == -1:
	FoundObject[2] = "Not Found"


#Print table, tells you what obstacles have been found in image and what obstacles are within the workspace of the arm and can be reached.
table = [["Obstacle 1", FoundObject[0], InWorkspace[0]],["Obstacle 2", FoundObject[1], InWorkspace[1]],["Obstacle 3", FoundObject[2], InWorkspace[2]]]
print tabulate(table,tablefmt="fancy_grid")

# Ask user which obstacle they want to grab, if they do not like any of them they can quit the program.
print "Which obstacle would you like the arm to grab? Enter the number."
print "If you do not like any of these options press 'q' to exit the program."


valid = False
while valid == False:
	command = raw_input()

	if command == 'q' or command == 'Q':
		print "Exiting Program"
		exit()
	elif command == '1':
		if inWorkspace[0]:
			#Set the goal to obstacle 1's angle position using inverse kinematics
			#print Obsticle1
			goal = Obsticle1Angles
			ObsticlesPart2 = [Obsticle2,Obsticle3]
			print "thetac", Obsticle1[3]
			# if Obsticle1[3]>40:
			# 	goalPart1,InWorkspace,thetac = armObject.calculateThetaC([Obsticle1[0],Obsticle1[1],Obsticle1[2]+3,0])
			# 	print "a"
			if abs(goal[0]<30):	
				goalPart1,InWorkspace,thetac = armObject.calculateThetaC([Obsticle1[0]-3,Obsticle1[1],Obsticle1[2],0],60)
				print "b"
			elif goal[0]>30:
				goalPart1,InWorkspace,thetac = armObject.calculateThetaC([Obsticle1[0]-3,Obsticle1[1]-3,Obsticle1[2],0],60)
				print "c"
			else:
				goalPart1,InWorkspace,thetac = armObject.calculateThetaC([Obsticle1[0]-3,Obsticle1[1]+3,Obsticle1[2],0],60)
				print "d"
			if InWorkspace == 0:
				print "Will not make it"
			#print goal
			print "Grabbing Obstacle 1..."
			valid = True
		else: 
			print "Sorry that Obstacle is not in the workspace the arm cannot go there."
	elif command == '2':
		if inWorkspace[1]:
			# set Goal
			#print Obsticle2
			goal = Obsticle2Angles
			print "thetac", Obsticle1[3]
			# if Obsticle1[3]>40:
			# 	goalPart1,InWorkspace,thetac = armObject.calculateThetaC([Obsticle1[0],Obsticle1[1],Obsticle1[2]+3,0])
			# 	print "a"
			if abs(goal[0]<30):	
				goalPart1,InWorkspace,thetac = armObject.calculateThetaC([Obsticle2[0]-3,Obsticle2[1],Obsticle2[2],0],60)
				print "b"
			elif goal[0]>30:
				goalPart1,InWorkspace,thetac = armObject.calculateThetaC([Obsticle2[0]-3,Obsticle2[1]-3,Obsticle2[2],0],60)
				print "c"
			else:
				goalPart1,InWorkspace,thetac = armObject.calculateThetaC([Obsticle2[0]-3,Obsticle2[1]+3,Obsticle2[2],0],60)
				print "d"
			if InWorkspace == 0:
				print "Will not make it"
			ObsticlesPart2 = [Obsticle1,Obsticle3]
			#print goal
			print "Grabbing Obstacle 2..."
			valid = True
		else: 
			print "Sorry that Obstacle is not in the workspace the arm cannot go there."
	elif command == '3':
		if inWorkspace[2]:
			# set Goal
			#print Obsticle3
			goal = Obsticle3Angles
			print "thetac", Obsticle1[3]
			# if Obsticle1[3]>40:
			# 	goalPart1,InWorkspace,thetac = armObject.calculateThetaC([Obsticle1[0],Obsticle1[1],Obsticle1[2]+3,0])
			# 	print "a"
			if abs(goal[0]<30):	
				goalPart1,InWorkspace,thetac = armObject.calculateThetaC([Obsticle3[0]-3,Obsticle3[1],Obsticle3[2],0],60)
				print "b"
			elif goal[0]>30:
				goalPart1,InWorkspace,thetac = armObject.calculateThetaC([Obsticle3[0]-3,Obsticle3[1]-3,Obsticle3[2],0],60)
				print "c"
			else:
				goalPart1,InWorkspace,thetac = armObject.calculateThetaC([Obsticle3[0]-3,Obsticle3[1]+3,Obsticle3[2],0],60)
				print "d"
			if InWorkspace == 0:
				print "Will not make it"
			ObsticlesPart2 = [Obsticle1,Obsticle2]
			#print goal
			print "Grabbing Obstacle 3..."
			valid = True
		else: 
			print "Sorry that Obstacle is not in the workspace the arm cannot go there."
	else:
		print "Sorry that command was not recognized."

# The next part of the code is to run the path planning program to find the path to the obstacle 
# This is done in two different parts. The first part is to move the arm close to the obstacle. 
# The gripper is than opened. The next part of the path planning calculates from that point to the object
# the gripper is then closed and the object picked up. The last part of the path moves the arm to  the
# 0,0,0,0 position. (straight out)
plannerObject = PathPlan()
ObsticlesPart1 = [Obsticle1,Obsticle2,Obsticle3]
start = [90,-20,30,0]
end = [0,-20,-20,20]
goalPart2 = goal

# Close Gripper
downlinkObject.sendValues(-2000,-2000,-2000,-2000)
#send path angles

print goalPart1
path1 = plannerObject.astar(start,goalPart1,ObsticlesPart1,2)
print path1

# Travel first part of path
print "Traveling first part of path."
for item in path1:
	downlinkObject.sendValues(item[0],item[1],item[2],item[3])

downlinkObject.sendValues(-2000,-2000,-2000,-2000)

length = len(path1) -1
path2 = plannerObject.astar(path1[length],goalPart2,ObsticlesPart2,2)
print path2

#Travel second part of path
print "Traveling second part of path."
for item in path2:
	downlinkObject.sendValues(item[0],item[1],item[2],item[3])

print "Hopefully grabbing object."
# Close Gripper
downlinkObject.sendValues(-1000,-1000,-1000,-1000)

length = len(path2) -1
path3 = plannerObject.astar(path2[length],end,ObsticlesPart2,2)
print path3
#path3 = path1
#path3.reverse()

# Go back to straight position
print "Moving arm to default position."
for item in path3:
	downlinkObject.sendValues(item[0],item[1],item[2],item[3])

time.sleep(2)

# Drop object
print "Dropping Object."
# Open Gripper
downlinkObject.sendValues(-2000,-2000,-2000,-2000)


# Send the found path to the manipulator. Each set of angles is sent to to the arm by serial
# communication. The arm controller than sends the corresponding duty cycle to the arm. Commands
# can also be sent to open and close the gripper of the arm. (-1000,X,X,X) , X for don't care, closes
# the gripper and (-2000,X,X,X) opens the gripper.






# # Open Gripper
# downlinkObject.sendValues(-2000,-2000,-2000,-2000)

# #Travel second part of path
# print "Traveling second part of path."
# for item in path2:
# 	downlinkObject.sendValues(item[0],item[1],item[2],item[3])

# print "Hopefully grabbing object."
# # Close Gripper
# downlinkObject.sendValues(-1000,-1000,-1000,-1000)


# # Go back to straight position
# print "Moving arm to default position."
# for item in path3:
# 	downlinkObject.sendValues(item[0],item[1],item[2],item[3])

# # Drop object
# print "Dropping Object."
# # Open Gripper
# downlinkObject.sendValues(-2000,-2000,-2000,-2000)

