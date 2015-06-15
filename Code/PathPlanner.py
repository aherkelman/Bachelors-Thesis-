import numpy as np 
import arm
from heapq import heappush, heappop
import time
import math

def print_timing(func):
    def wrapper(*arg):
        t1 = time.time()
        res = func(*arg)
        t2 = time.time()
        print '%s took %0.3f ms' % (func.func_name, (t2-t1)*1000.0)
        return res
    return wrapper
class PathPlan:

	def __init__(self):
		# Start and desired end position in angles
		# Four values correspond to the 4 different joint angles
		#self.obsticles() = []
		self.startPos = np.zeros(4)
		self.endPos = np.zeros(4)
		# Calculated Path
		#self.path
		# Adjustment values
		self.stepSize = 3
		self.obsticleCost = 100
		self.goalCost = 2
		self.toHereCost = 1
		self.movements = [
						(self.stepSize,0,0,0), (0,self.stepSize,0,0), (0,0,self.stepSize,0), (0,0,0,self.stepSize), (-self.stepSize,0,0,0),
						(0,-self.stepSize,0,0), (0,0,-self.stepSize,0), (0,0,0,-self.stepSize).....]
						#Some excluded for length reasons
	def distance3(self,p, q):
		return np.sqrt((p[0]-q[0])**2 + (p[1]-q[1])**2 + (p[2]-q[2])**2)
	def distance4(self,p, q):
		return np.sqrt((p[0]-q[0])**2 + (p[1]-q[1])**2 + (p[2]-q[2])**2 + (p[3]-q[3])**2)
	def square1(self,p):
		s = p[0,0]**2 + p[1,0]**2 + p[2,0]**2
		return s
	def square2(self,p):
		s = p[0]**2 + p[1]**2 + p[2]**2
		return s
	def mult(self,p,q):
		m = [p[0,0]*q[0,0],p[1,0]*q[1,0],p[2,0]*q[2,0]]
		return m

	@print_timing
	def astar(self, start, goal, obsticles,pathPart):
		# Calculate path from inputs
		armObject = arm.Manipulator()
		front = [ (0.001,0.001, start, None) ]
		# In the beginning, no cell has been visited.
		visited = []
		x = 0
		# Also, we use a dictionary to remember where we came from.
		came_from = []
		if pathPart == 1:
			atGoal = 3
		else:
			atGoal = 0.5
		Tjoint1,Tjoint2,Tjoint3,Tjoint4,GoalxyzPos = armObject.forwardKinematics(goal)
		# While there are elements to investigate in our front.
		print "Planning Path"
		while front:
			# Get smallest item and remove from front.
			element = heappop(front)
			# Check if this has been visited already.
			# CHANGE 01_e: use the following line as shown.
			total_cost, cost, pos, previous = element
			# Now it has been visited. Mark with cost.
			for item in visited:
				if item == pos:
					continue #return to top of while loop
			visited.append(pos)
			#print pos
			# Also remember that we came from previous when we marked pos.
			came_from.append(previous)
			x=x+1
			Tjoint1,Tjoint2,Tjoint3,Tjoint4,xyzPos = armObject.forwardKinematics(pos)

			# Check if the goal has been reached.
			if (self.distance3(xyzPos,GoalxyzPos)<=atGoal):
			#if (self.distance4(pos,goal)<=12):
				break  # Finished!
			# Check all neighbors.
			for d1, d2, d3, d4 in self.movements:
				# Determine new position and check bounds.
				new_theta1 = pos[0] + d1
				if new_theta1> 90 | new_theta1<-90:
					continue
				new_theta2 = pos[1] + d2
				if new_theta2>120 | new_theta2 <-50:
					continue
				new_theta3 = pos[2] + d3
				if new_theta3>120 | new_theta3 <-55:
					continue
				new_theta4 = pos[3] + d4
				if new_theta4>120 | new_theta4 <-55:
					continue
				# Add to front if: not visited before and no obstacle.
				new_pos = (new_theta1,new_theta2,new_theta3,new_theta4)
				for item in visited:
					if item == new_pos:
						continue #return to top of while loop
				# calculate cost to get to new point
				cost_to_new_pos = cost + self.distance4(new_pos,pos)
				# estimate cost to get to goal
				estimate_cost_to_goal = self.distance4(new_pos,goal)
				# add cost to avoid obsticles
				obsticle_cost = 0
				floor_cost = 0
				Tjoint1,Tjoint2,Tjoint3,Tjoint4,xyzPos = armObject.forwardKinematics(new_pos)

				#print 'd', obsticle_distance
				for item in obsticles:
					obsticle_cost = obsticle_cost+1/(self.distance3(item,xyzPos))
	
				heappush(front,(floor_cost+self.toHereCost*cost_to_new_pos+self.goalCost*estimate_cost_to_goal+self.obsticleCost*obsticle_cost,cost_to_new_pos,new_pos,pos))
		# Reconstruct path, starting from goal.
		path = []
		if (self.distance3(xyzPos,GoalxyzPos)<=atGoal):  # If we reached the goal, unwind backwards.
		#if (self.distance4(pos,goal)<=12):
			while pos:
				for item in visited:
					if item == pos:
						break #return to top of while loop
				index = visited.index(item)
				pos = came_from[index]
				if pos:
					path.append(pos)
			path.reverse()  # Reverse so that path is from start to goal.
		return path



