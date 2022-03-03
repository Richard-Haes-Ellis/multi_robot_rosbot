#!/usr/bin/env python
#import rospy

import rclpy
from rclpy.node import Node
import math
from math import pow, atan2, sqrt
from geometry_msgs.msg import Twist, TwistStamped, PoseStamped
from geometry_msgs.msg import Pose, PoseWithCovariance
from nav_msgs.msg import Odometry, OccupancyGrid, Path
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

fig = plt.figure()
ax = fig.add_subplot(111)

class Planner(Node):
	def __init__(self):
     
		super().__init__('Pathplanner')        
		self.subscription_odom = self.create_subscription(Odometry,'/rosbot1/odom',self.update_pose,10)
		self.subscription_odom  # prevent unused variable warning
		self.subscription_map = self.create_subscription(OccupancyGrid,'/rosbot1/map',self.update_map,10)
		self.subscription_map  # prevent unused variable warning 
		self.subscription_vel = self.create_subscription(PoseStamped,'/rosbot1/cmd_vel',self.update_vel,10)
		self.subscription_vel  # prevent unused variable warning
  
		self.subscription_goal = self.create_subscription(PoseStamped,'/rosbot1/current_goal',self.calcula_ruta,10)
		self.subscription_goal  # prevent unused variable warning
  
		self.publisher_path = self.create_publisher(Path, '/rosbot1/current_path',10)
		self.publisher_path  # prevent unused variable warning

  
        
		self.pose_cov = PoseWithCovariance()
		self.pose = Pose()
		self.mapa = OccupancyGrid()
		self.cmd_vel = Twist()
		self.goal = PoseStamped()
		self.ruta = Path()

		self.map_origin = Pose()

	def update_map(self, data):  #Actualiza informacion del mapa
		self.mapa = data
		self.map_width = self.mapa.info.width
		self.map_height = self.mapa.info.height
		self.map_origin = self.mapa.info.origin
		self.map_data = self.mapa.data
		self.map_resolution = self.mapa.info.resolution
  
		offset_x = self.map_origin.position.x #posicion en metros de la celda (0,0)
		offset_y = self.map_origin.position.y

		dim_x = self.map_width
		dim_y = self.map_height

		print("Dimensiones del mapa: ",dim_x, '*', dim_y)
		print("Origen del mapa en: ", [offset_x, offset_y])
		print("Resolucion del mapa: ", self.map_resolution)

		print_list = []


		self.matriz_mapa = np.zeros([dim_y, dim_x])
  
		for j in range(dim_x):
			for i in range(dim_y):
				self.matriz_mapa [i][j] = self.map_data[j*dim_y + i]
				if(self.matriz_mapa[i][j] > 20):
					self.matriz_mapa[i][j] = 1
				else:
					self.matriz_mapa[i][j] = 0
    
  
	def update_vel(self, data):
		self.cmd_vel = data   
	def update_pose(self, data):
		self.pose_cov = data.pose
		self.pose = self.pose_cov.pose
		self.pose.position.x = round(self.pose.position.x, 2)
		self.pose.position.y = round(self.pose.position.y, 2)
  
  
		#PRUEBAS para ver las conversiones
		print("Odom indica que el robot está en: ", [self.pose.position.x, self.pose.position.y])
  
		current_cell_x = math.trunc(self.pose.position.x/self.map_resolution)
		current_cell_y = math.trunc(self.pose.position.y/self.map_resolution)
  
		print("El robot está en la celda: ", [current_cell_x, current_cell_y])
	
		orientation = [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w]
       

	def compute_path(self, start_cell, goal_cell):
		"""Compute path."""
		path = []

		X = []
		Y = []
	
		######################
		# 		   A*        #
		######################
		class cell ():
			def __init__(self, x, y, gn, hn, fn, parent) :
				self.x = x
				self.y = y
				self.gn = gn
				self.hn = hn
				self.fn = fn
				self.parent = parent

			def calculafn(self) :
				self.hn = abs(goal_cell[1] - self.x) + abs(goal_cell[0] - self.y)	#Manhattan
				# self.hn = sqrt(pow((goal_cell[2] - self.x), 2) + pow((goal_cell[1] - self.y), 2) + pow((goal_cell[0] - self.z), 2))	#Euclidea
				self.fn = self.gn + self.hn

			
		lista_abierta = []
		lista_cerrada = []

		# Paso 0: incluyo celda origen a la lista abierta
		celda = cell(start_cell[1], start_cell[0], 0, 0, 0, None)
		celda.calculafn()
		lista_abierta.append(celda)
		
		
		# Paso 1: ordeno lista abierta, saco primer elemento y lo meto en la cerrada 

		lista_abierta.sort(key=lambda x: x.fn)


		while (lista_abierta != False) :	#solo mientras queden elementos en la lista abierta
			celda = lista_abierta.pop(0)
			lista_cerrada.append(celda)
			if (celda.x == goal_cell[1] and celda.y == goal_cell[0]):	#Paso 2: he encontrado la meta, salgo
				break
				

			# Paso 3: calculo vecinos
			neighbors_list = []
			neighbors_list.append([celda.y + 1, celda.x])		
			neighbors_list.append([celda.y, celda.x + 1])
			neighbors_list.append([celda.y - 1, celda.x])
			neighbors_list.append([celda.y, celda.x - 1])

			# Paso 4: compruebo si existe en alguna lista
			for vecino in neighbors_list :		
				if (self.matriz_mapa[vecino[1],vecino[0]] == 0) :	#solo busco si la celda esta libre de obstaculos 
					found = 0
					for objeto in lista_abierta : #si esta en la abierta, actualizo gn y reordeno
						if(objeto.x == vecino[1] and objeto.y == vecino[0]) :
							found = 1
							g_n = celda.gn + 1
							if (g_n < objeto.gn) :
								objeto.gn = g_n
								objeto.calculafn()
								objeto.parent = celda
								lista_abierta.sort(key=lambda x: x.fn) #reordeno la lista
							
							
					for objeto in lista_cerrada : #si esta en la cerrada, hago lo mismo
						if(objeto.x == vecino[1] and objeto.y == vecino[0]) :
							found = 1
							g_n = celda.gn + 1
							if (g_n < objeto.gn) :
								objeto.gn = g_n
								objeto.calculafn()
								objeto.parent = celda
								lista_abierta.sort(key=lambda x: x.fn) #reordeno la lista
			
					if(found==0) :			#si no esta en ninguna lista, lo incluyo y reordeno
						neighbor = cell(vecino[1],vecino[0], celda.gn + 1, 0, 0, celda)
						neighbor.calculafn()
						lista_abierta.append(neighbor)
						lista_abierta.sort(key=lambda x: x.fn)

		while(celda.parent != None):			#Una vez encontrada la meta, obtengo el camino
			path.append([celda.y, celda.x])
			celda = celda.parent

		path.append(start_cell)
		path.reverse()

		
		######################
		# End A*             #
		######################

		# Print path
		x = []
		y = []
		
		pathArray=[]
		path = Path()
		pose_path = Pose()
    
		for point in path:
			pose_path.position.x = point[1] #Path de celdas
			pose_path.position.y = point[0] #Path de celdas
   
			pose_path.position.x = pose_path.position.x*self.map_resolution + self.map_origin.position.x #Path en coordenadas absolutas
			pose_path.position.y = pose_path.position.y*self.map_resolution + self.map_origin.position.y

			pathArray.append(pose_path)
  
		path.poses = pathArray
		ax.scatter(x, y, c='g', marker='x')
		plt.show()		
		return path


	def calcula_ruta(self,data):
		"""Moves the robot to the goal."""
		
		self.goal = data
		current_cell_x = math.trunc((self.pose.position.x - self.map_origin.position.x)/self.map_resolution) + math.trunc(self.map_origin.position.x/self.map_resolution) #odom respecto a la matriz de ocupacion (desplaz en celdas entre origen del mapa y robot)
		current_cell_y = math.trunc((self.pose.position.y - self.map_origin.position.y)/self.map_resolution) + math.trunc(self.map_origin.position.y/self.map_resolution)

		current_cell = [current_cell_y, current_cell_x]
		print("Partiendo de celda", current_cell)


		goal_x= math.trunc((self.goal.pose.position.x - self.map_origin.position.x)/self.map_resolution) + math.trunc(self.map_origin.position.x/self.map_resolution) #recibo goal en coordenadas ABSOLUTAS del mapa
		goal_y= math.trunc((self.goal.pose.position.y - self.map_origin.position.y)/self.map_resolution) + math.trunc(self.map_origin.position.y/self.map_resolution)

		goal_cell = [goal_y, goal_x]
  
		print("Celda meta", goal_cell)


		self.ruta = self.compute_path(current_cell,goal_cell)
  
		self.publisher_.publish(self.ruta)



if __name__ == '__main__':
	rclpy.init(args=None)
	x = Planner()
	rclpy.spin(x)
  
