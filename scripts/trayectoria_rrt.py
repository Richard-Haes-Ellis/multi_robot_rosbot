#!/usr/bin/env python
import rospy
import math
from math import pow, atan2, sqrt
from geometry_msgs.msg import TwistStamped, PoseStamped
from geometry_msgs.msg import Pose, PoseWithCovariance
from nav_msgs.msg import Odometry
from uav_abstraction_layer.srv import GoToWaypoint, TakeOff
import tf.transformations
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import random




fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

class object:
    def __init__(self,name,lim_x,lim_y,lim_z):
        self.name = name
        self.lim_x = lim_x
        self.lim_y = lim_y
        self.lim_z = lim_z

class branch:
    def __init__(self,coord,father,cost):
        self.coord = coord
        self.father = father
        self.cost = cost


class Planner:

    def __init__(self):
        
	self.pose_subscriber = rospy.Subscriber('/ual/odom', Odometry, self.update_pose)
	self.pose_stamped = PoseStamped() 
	self.pose_cov = PoseWithCovariance()
	self.pose = Pose()
	
        
    def update_pose(self, data):
        """Callback function which is called when a new message of type Pose is
        received by the subscriber."""

    	self.pose_stamped.pose = data.pose
	self.pose_cov = self.pose_stamped.pose
	self.pose = self.pose_cov.pose
	self.pose.position.x = round(self.pose.position.x, 4)
	self.pose.position.y = round(self.pose.position.y, 4)
	self.pose.position.z = round(self.pose.position.z, 4)

        
    	orientation = [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w]
        
    def compute_path(self, start_cell, goal_cell, map_sel):
        """Compute path."""
        path = []

        

	if map_sel == 1:
		ob1 = object("HOUSE",[-3.5, 11.5],[-15.5, -3.5],[0, 7.5])
		ob2 = object("TABLE",[2.5, 6.5],[-1.5, 1.5],[0, 2])
		ob3 = object("TREE",[-6.5, 6.5],[3.3, 7.7],[0, 5.5])
		ob4 = object("WALL",[-5.5, -3.5],[-3.5, 5.5],[0, 3])
		ob5 = object("LIGHT 1",[-9.5, -7.5],[-0.5, 1.3],[0, 5])
		ob6 = object("LIGHT 2",[-13.7, -7.5],[-0.5, 1.3],[5, 6.5])

		object_list = [ob1, ob2, ob3, ob4, ob5, ob6]

	if map_sel == 2:
		ob1 = object("AMBULANCE",[1.5, 10.5],[6.5, 10.5],[0, 3.5])
		ob2 = object("AMB BARRIER",[4.5, 9.5],[5.5, 7.5],[0, 1.5])
		ob3 = object("WALL 1",[-1.5, 0.5],[1.5, 10.5],[0, 3.5])
		ob4 = object("WALL 2",[-8.5, 0.5],[1.5, 3.5],[0, 3.5])
		ob5 = object("TREE",[-5.5, -0.5],[-1.5, 3.5],[0, 6.5])
		ob6 = object("HUMMER",[-9.5, -5.5],[-8.5, -1.5],[0, 3.5])
		ob7 = object("FIREMAN CAR",[-3.5, 10.5],[-10.5, -6.5],[0, 4.5])
		ob8 = object("HUMAN 1",[-3.5, -1.5],[-5.5, -3.5],[0, 2.5])
		ob9 = object("HUMAN 2",[1.5, 3.5],[-4.5, -2.5],[0, 2.5])
		ob10 = object("BOX 1",[3.5, 6.5],[-2.5, -0.5],[0, 3.5])
		ob11 = object("BOX 2",[6.5, 10.5],[-3.5, -1.5],[0, 3.5])
		ob12 = object("BOX 3",[5.5, 11.5],[-0.5, 1.5],[0, 2.5])
		ob13 = object("BOX 4",[8.5, 10.5],[0.5, 5.5],[0, 2.5])
		ob14 = object("BOX 5",[5.5, 8.5],[2.5, 4.5],[0, 4.5])
		ob15 = object("FOUNTAIN",[2.5, 5.5],[2.5, 5.5],[0, 3.5])
		ob16 = object("CONTAINER 1",[-10.5, -7.5],[5.5, 9.5],[0, 2.5])
		ob17 = object("CONTAINER 2",[-6.5, -2.5],[6.5, 9.5],[0, 2.5])


		object_list = [ob1, ob2, ob3, ob4, ob5, ob6, ob7, ob8, ob9, ob10, ob11, ob12, ob13, ob14, ob15, ob16, ob17]



	offset_x = -20
	offset_y = -20
	def evaluation(ob_ev,x,y,z):
			if (x <= ob_ev.lim_x[1] and x >= ob_ev.lim_x[0])\
			and (y <= ob_ev.lim_y[1] and y >= ob_ev.lim_y[0])\
			and (z <= ob_ev.lim_z[1] and z >= ob_ev.lim_z[0]):
				return 1
			else:
				return 0

	dim_x = 40
	dim_y = 40
	dim_z = 30


	print_list = []

	for z in range(dim_z):
			for y in range(dim_y):
				for x in range(dim_x):
						for j in range(len(object_list)):
							if evaluation(object_list[j],x +offset_x,y +offset_y, z) == 1:
									print_list.append([x +offset_x, y +offset_y, z])
									break

	X = []
	Y = []
	Z = []

	for n in range(len(print_list)):
		X.append(print_list[n][0])
		Y.append(print_list[n][1])
		Z.append(print_list[n][2])

	ax.scatter(X, Y, Z, c='r', marker='o')

	ax.set_xlabel('X Label')
	ax.set_ylabel('Y Label')
	ax.set_zlabel('Z Label')

	for element in object_list:
		ax.text(element.lim_x[0],element.lim_y[0],element.lim_z[0],element.name,color='blue')

	ax.scatter(0, 0, 0, c='y', marker='x')

	

	######################
	# TODO: Implement RRT* #
	######################

	
	Ps = [start_cell[2],start_cell[1],start_cell[0]]
	
	Pe = [goal_cell[2],goal_cell[1],goal_cell[0]]
	

	
	D = 1.0
	

	
	dist_0 = 9999

	# tree es un array de objetos tipo branch(coor,father,cost)
	tree = []
	# Posicion inicial de tree
	tree.append(branch(Ps,Ps,0))
	# Condiciones iniciales para finalizar el arbol y de colision
	endTree = False
	colision = False

	# vectores auxiliares
	tree_res = []
	x_tree = []
	y_tree = []
	z_tree = []
	coord_list = []
	father_list = []

	# variables que representa cada iteracion para tener una idea
	# de como va el programa
	iter = 0

	while endTree == False:
		Pr = [random.uniform(-dim_x/2, dim_x/2), random.uniform(-dim_y/2, dim_y/2), random.uniform(-dim_z/2, dim_z/2)]
		while tree.count(Pr) != 0:
			Pr = [random.uniform(-dim_x/2, dim_x/2), random.uniform(-dim_y/2, dim_y/2), random.uniform(-dim_z/2, dim_z/2)]

		for point in tree:
			dist_1 = math.sqrt((Pr[0]-point.coord[0])**2 +(Pr[1]-point.coord[1])**2 +(Pr[2]-point.coord[2])**2)
			if dist_1 < dist_0:
				Pi = point.coord
				dist_0 = dist_1

		Vd = [(Pr[0]-Pi[0])*(D/dist_0), (Pr[1]-Pi[1])*(D/dist_0), (Pr[2]-Pi[2])*(D/dist_0)]
		Pd = [Vd[0]+Pi[0], Vd[1]+Pi[1], Vd[2]+Pi[2]]    

		for j in range(len(object_list)):
			if evaluation(object_list[j], Pd[0], Pd[1], Pd[2]) == 1:
				colision = True
				break

		if colision == False and Pd[2] > 0:
			tree.append(branch(Pd, Pi, dist_0))

			dist_2 = math.sqrt((Pe[0]-Pd[0])**2 +(Pe[1]-Pd[1])**2 +(Pe[2]-Pd[2])**2)
			if dist_2 <= D:
				tree.append(branch(Pe, Pd, dist_2))
				endTree = True
		else:
			colision = False
		
		dist_0 = 9999
		iter = iter + 1
		print("Iteracion numero: ", iter)

	print("FIN SEARCH")
	# Hemos terminado de recopilar los puntos en tree
	# ahora hace falta ordenarlos

	for point in tree:
		coord_list.append(point.coord)
		father_list.append(point.father)
		

	s_point = Pe
	while True:
		tree_res.append(s_point)
		if s_point == Ps:
			break
		s_point = father_list[coord_list.index(s_point)]

	tree_res.reverse()

	for point in tree_res:
		x_tree.append(point[0])
		y_tree.append(point[1])
		z_tree.append(point[2])

	ax.plot(x_tree, y_tree, z_tree, c='b', marker='x')

	print("Arbol sin suavizar: ")
	print(tree_res)
	# tree_res representa el arbol tree con solo los puntos de interes
	# y ya organizado de inicio a final

	############## SUAVIZADO ##############

	# Esta funcion se encarga de revisar de si hay un objeto entre dos puntos dados
	# evaluando cada 0.1 en distancia entre ellos

	step = 0.1
	def rute_free(start, goal):
		m = math.sqrt((goal[0]-start[0])**2 +(goal[1]-start[1])**2 +(goal[2]-start[2])**2)
		result = True
		for s in np.arange(step, m, step):
			v = [(goal[0]-start[0])*(s/m), (goal[1]-start[1])*(s/m), (goal[2]-start[2])*(s/m)]
			p = [v[0]+start[0], v[1]+start[1], v[2]+start[2]]
			for j in range(len(object_list)):
				if evaluation(object_list[j],p[0], p[1], p[2]) == 1:
					result = False
					break
		return result

	
	tree_res_s = []
	tree_res_s.append(tree_res[0])
	suav_point = tree_res[0]
	while True:
		index = tree_res.index(suav_point)
		for n in np.arange(index+1,len(tree_res)):
			prox_point = tree_res[n]
			if rute_free(suav_point, prox_point) == True:
				next_point = prox_point

		
		print("Si esto se repite sin fin reintenta de nuevo")
		print(next_point, n)

		tree_res_s.append(next_point)
		suav_point = next_point
		if next_point == tree_res[-1]:
				break

	
	######################
	# End RRT*             #
	######################

	#Print Path#

	x_tree_s = []
	y_tree_s = []
	z_tree_s = []
	for point in tree_res_s:
		x_tree_s.append(point[0])
		y_tree_s.append(point[1])
		z_tree_s.append(point[2])

	ax.plot(x_tree_s, y_tree_s, z_tree_s, c='g', marker='o')



	print("Arbol suavizado:")
	print(tree_res_s)
	print("FIN")

	plt.show()
	
	return tree_res_s

    def goto(self):
        """Moves the robot to the goal."""

        goal_pose = Pose()
        # Get the input from the user.
        # Test with -0.5,3.5 meters
	while True:
		data = input("Map, type 1 for house, type 2 for crowded square: ")
		if data == 1 or data == 2:
			map_sel = data
			break
		else:
			print("You must type either 1 or 2")

	if(map_sel == 1):
		while True:
			while True:
				data = input("Set your x goal: ")
				if data <= 20 and data >= -20:
					goal_pose.position.x = data
					break
				else:
					print("x goal must be between -20, 20")

			while True:
				data = input("Set your y goal: ")
				if data <= 20 and data >= -20:
					goal_pose.position.y = data
					break
				else:
					print("y goal must be between -20, 20")

			while True:
				data = input("Set your z goal: ")
				if data > 0:
					goal_pose.position.z = data
					break
				else:
					print("z goal must be greater than 0")

			if (goal_pose.position.x >= -3.5 and goal_pose.position.x <= 11.5 and goal_pose.position.y >= -15.5 and goal_pose.position.y <= -3.5 and goal_pose.position.z >= 0 and goal_pose.position.z <= 11.5) \
				or (goal_pose.position.x >= 2.5 and goal_pose.position.x <= 6.5 and goal_pose.position.y >= -1.5 and goal_pose.position.y <= 1.5 and goal_pose.position.z >= 0 and goal_pose.position.z <= 2) \
				or (goal_pose.position.x >= -6.5 and goal_pose.position.x <= 6.5 and goal_pose.position.y >= 3.3 and goal_pose.position.y <= 7.7 and goal_pose.position.z >= 0 and goal_pose.position.z <= 5.5) \
				or (goal_pose.position.x >= -4.3 and goal_pose.position.x <= -3.7 and goal_pose.position.y >= -3.5 and goal_pose.position.y <= 5.5 and goal_pose.position.z >= 0 and goal_pose.position.z <= 3) \
				or (goal_pose.position.x >= -9.5 and goal_pose.position.x <= -7.5 and goal_pose.position.y >= -0.5 and goal_pose.position.y <= 1.3 and goal_pose.position.z >= 0 and goal_pose.position.z <= 5.5) \
				or (goal_pose.position.x >= -13.7 and goal_pose.position.x <= -7.5 and goal_pose.position.y >= -0.5 and goal_pose.position.y <= 1.3 and goal_pose.position.z >= 5 and goal_pose.position.z <= 6.5):

				print("Goal point is inside an object- try another one")
				
			else:
				break
	
	if(map_sel == 2):
		while True:
			while True:
				data = input("Set your x goal: ")
				if data <= 20 and data >= -20:
					goal_pose.position.x = data
					break
				else:
					print("x goal must be between -20, 20")

			while True:
				data = input("Set your y goal: ")
				if data <= 20 and data >= -20:
					goal_pose.position.y = data
					break
				else:
					print("y goal must be between -20, 20")

			while True:
				data = input("Set your z goal: ")
				if data > 0:
					goal_pose.position.z = data
					break
				else:
					print("z goal must be greater than 0")

			if (goal_pose.position.x >= 1.5 and goal_pose.position.x <= 10.5 and goal_pose.position.y >= 6.5 and goal_pose.position.y <= 10.5 and goal_pose.position.z >= 0 and goal_pose.position.z <= 3.5) \
				or (goal_pose.position.x >= 4.5 and goal_pose.position.x <= 9.5 and goal_pose.position.y >= 5.5 and goal_pose.position.y <= 7.5 and goal_pose.position.z >= 0 and goal_pose.position.z <= 1.5) \
				or (goal_pose.position.x >= -1.5 and goal_pose.position.x <= 0.5 and goal_pose.position.y >= 1.5 and goal_pose.position.y <= 10.5 and goal_pose.position.z >= 0 and goal_pose.position.z <= 3.5) \
				or (goal_pose.position.x >= -8.5 and goal_pose.position.x <= 0.5 and goal_pose.position.y >= 1.5 and goal_pose.position.y <= 3.5 and goal_pose.position.z >= 0 and goal_pose.position.z <= 3.5) \
				or (goal_pose.position.x >= -5.5 and goal_pose.position.x <= -0.5 and goal_pose.position.y >= -1.5 and goal_pose.position.y <= 3.5 and goal_pose.position.z >= 0 and goal_pose.position.z <= 6.5) \
				or (goal_pose.position.x >= -9.5 and goal_pose.position.x <= -5.5 and goal_pose.position.y >= -8.5 and goal_pose.position.y <= -1.5 and goal_pose.position.z >= 0 and goal_pose.position.z <= 3.5) \
				or (goal_pose.position.x >= -3.5 and goal_pose.position.x <= 10.5 and goal_pose.position.y >= -10.5 and goal_pose.position.y <= -6.5 and goal_pose.position.z >= 0 and goal_pose.position.z <= 4.5) \
				or (goal_pose.position.x >= -3.5 and goal_pose.position.x <= -1.5 and goal_pose.position.y >= -5.5 and goal_pose.position.y <= -3.5 and goal_pose.position.z >= 0 and goal_pose.position.z <= 2.5) \
				or (goal_pose.position.x >= 1.5 and goal_pose.position.x <= 3.5 and goal_pose.position.y >= -4.5 and goal_pose.position.y <= -2.5 and goal_pose.position.z >= 0 and goal_pose.position.z <= 2.5) \
				or (goal_pose.position.x >= 3.5 and goal_pose.position.x <= 6.5 and goal_pose.position.y >= -2.5 and goal_pose.position.y <= -0.5 and goal_pose.position.z >= 0 and goal_pose.position.z <= 3.5) \
				or (goal_pose.position.x >= 6.5 and goal_pose.position.x <= 10.5 and goal_pose.position.y >= -3.5 and goal_pose.position.y <= -1.5 and goal_pose.position.z >= 0 and goal_pose.position.z <= 3.5) \
				or (goal_pose.position.x >= 5.5 and goal_pose.position.x <= 11.5 and goal_pose.position.y >= -0.5 and goal_pose.position.y <= 1.5 and goal_pose.position.z >= 0 and goal_pose.position.z <= 2.5) \
				or (goal_pose.position.x >= 8.5 and goal_pose.position.x <= 10.5 and goal_pose.position.y >= 0.5 and goal_pose.position.y <= 5.5 and goal_pose.position.z >= 0 and goal_pose.position.z <= 2.5) \
				or (goal_pose.position.x >= 5.5 and goal_pose.position.x <= 8.5 and goal_pose.position.y >= 2.5 and goal_pose.position.y <= 4.5 and goal_pose.position.z >= 0 and goal_pose.position.z <= 4.5) \
				or (goal_pose.position.x >= 2.5 and goal_pose.position.x <= 5.5 and goal_pose.position.y >= 2.5 and goal_pose.position.y <= 5.5 and goal_pose.position.z >= 0 and goal_pose.position.z <= 3.5) \
				or (goal_pose.position.x >= -10.5 and goal_pose.position.x <= -7.5 and goal_pose.position.y >= 5.5 and goal_pose.position.y <= 9.5 and goal_pose.position.z >= 0 and goal_pose.position.z <= 2.5) \
				or (goal_pose.position.x >= -6.5 and goal_pose.position.x <= -2.5 and goal_pose.position.y >= 6.5 and goal_pose.position.y <= 9.5 and goal_pose.position.z >= 0 and goal_pose.position.z <= 2.5) :

				print("Goal point is inside an object- try another one")
				
			else:
				break

		

	current_z= (self.pose.position.z)
	current_y= (self.pose.position.y)
	current_x= (self.pose.position.x)

	current_cell = [current_z, current_y, current_x]

	goal_z= (goal_pose.position.z)
	goal_y= (goal_pose.position.y)
	goal_x= (goal_pose.position.x)

	goal_cell = [goal_z, goal_y, goal_x]

	path = self.compute_path(current_cell,goal_cell, map_sel)

		

	for point in path:
			# TODO: Call service GoTo
		
		try:
			goto_service=rospy.ServiceProxy('/ual/go_to_waypoint',GoToWaypoint)

			waypoint = PoseStamped()		
			waypoint.header.seq = 0 
			waypoint.header.stamp.secs = 0 
			waypoint.header.stamp.nsecs = 0
			waypoint.header.frame_id = ''
			waypoint.pose.position.x = point[0]
			waypoint.pose.position.y = point[1]
			waypoint.pose.position.z = point[2]
			waypoint.pose.orientation.x = 0
			waypoint.pose.orientation.y = 0
			waypoint.pose.orientation.z = 0
			waypoint.pose.orientation.w = 0
		
			goto_service(waypoint,1)

		except rospy.ServiceException, e:
			print "Service goto call failed: %s"%e
			pass


        
if __name__ == '__main__':
    try:
        rospy.init_node('robot_planner', anonymous=True)

        x = Planner()
        x.goto()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
