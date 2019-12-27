#!/usr/bin/env python

#date july-20-2019
#Written by Chaitanya
#version 1.2

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PointStamped,Vector3Stamped
from std_msgs.msg import Float32, Bool
from dji_sdk.srv import DroneTaskControl, SDKControlAuthority, SetLocalPosRef
#import pid
from TrajectoryGenerator import TrajectoryGenerator

class DJI_min_jerk_nav:


	def __init__(self):

		rospy.init_node('DJI_min_jerk_controller',anonymous=True)# Creates a node with name 's900_controller'  # unique node (using anonymous=True).

		self.velocity_publisher = rospy.Publisher('/dji1_sdk/flight_control_setpoint_ENUvelocity_yawrate', Joy,queue_size=10)# Publisher which will publish to the topic

		self.drone_position = PointStamped()

		self.drone_velocity = Vector3Stamped()
		
		self.pose_subscriber = rospy.Subscriber('dji1_sdk/local_position', PointStamped,self.update_pose)# when a message of type Pose is received.

		#self.imu_position = PointStamped()

		#self.imu = rospy.Subscriber('/dji_sdk/acceleration_ground_fused',Vector3Stamped,self.callback_imu)
		
		#self.lidar_subscriber = rospy.Subscriber('/lidar', Float32, self.lidar_callback)
		
		#self.epm_publisher = rospy.Publisher('/epm_toggle', Bool, queue_size = 1)

		self.velocity_subscriber = rospy.Subscriber('/dji1_sdk/velocity',Vector3Stamped,self.update_velocity)

		self.rate = rospy.Rate(10)

		# Proportional coefficients
		self.Kp_x = 0.4
		self.Kp_y = 0.4
		self.Kp_z = 0.4
		
		# Derivative coefficients
		self.Kd_x = 0.25
		self.Kd_y = 0.25
		self.Kd_z = 0.25

		#Co-efficients
		self.x_c = []
		self.y_c = []
		self.z_c = []

		#increment tragectory time
		self.increment_trag_time = 0.1

		#trag_time
		self.trag_time = 0

		#wPoint_count
		self.wPoint_count = 0

	def update_pose(self, drone_position_temp):

		"""Callback function which is called when a new message of type Pose is received by the subscriber."""

		self.drone_position.point.x = round(drone_position_temp.point.x, 4)
		self.drone_position.point.y = round(drone_position_temp.point.y, 4)
		self.drone_position.point.z = round(drone_position_temp.point.z, 4)

	def update_velocity(self, drone_velocity_temp):

		#print(drone_velocity_temp)

		self.drone_velocity.vector.x = round(drone_velocity_temp.vector.x,4)
		self.drone_velocity.vector.y = round(drone_velocity_temp.vector.y,4)
		self.drone_velocity.vector.z = round(drone_velocity_temp.vector.z,4)

	def didwPointReached(self,des_wpoint_param,tolarance,drone_position):

		tolaranceX = tolarance[0]
		tolaranceY = tolarance[1]
		tolaranceZ = tolarance[2]

		present_X = drone_position.x
		present_Y = drone_position.y
		present_Z = drone_position.z

		presentTolaranceX = des_wpoint_param[0] - present_X
		presentTolaranceY = des_wpoint_param[1] - present_Y
		presentTolaranceZ = des_wpoint_param[2] - present_Z

		if (abs(presentTolaranceX) < tolaranceX and abs(presentTolaranceY) < tolaranceY and abs(presentTolaranceZ) < tolaranceZ):
			return True
		else:
			return False


	def move2wpoint(self,wplist):

		drone_pose = self.drone_position.point

		init_prese_point_x = drone_pose.x
		init_prese_point_y = drone_pose.y
		init_prese_point_z = drone_pose.z

		init_des_waypoint_x = wplist[self.wPoint_count][0]
		init_des_waypoint_y = wplist[self.wPoint_count][1]
		init_des_waypoint_z = wplist[self.wPoint_count][2]

		init_prese_point = [init_prese_point_x,init_prese_point_y,init_prese_point_z]
		init_desired_wpoint = [init_des_waypoint_x,init_des_waypoint_y,init_des_waypoint_z] 

		print('init_present',init_prese_point)
		print('init_des',init_desired_wpoint)
		print('time',self.trag_time)


		traj = TrajectoryGenerator(init_prese_point, init_desired_wpoint , self.trag_time + 0.1 ,[0,0,0],[0,0,0])

		traj.solve()

		print('',)

		self.x_c = traj.x_c
		self.y_c = traj.y_c
		self.z_c = traj.z_c

		print('co-efficients',self.x_c,self.y_c,self.z_c)

		vel_wpoint_cmd = Joy()
		waypoints_parm_length = len(wplist)

		def wpoint_increment():

			pr_wp = wplist[self.wPoint_count]

			tolarance = [0.4,0.4,0.4]

			drone_pose = self.drone_position.point
			
			wPoint_reached = self.didwPointReached(pr_wp,tolarance,drone_pose)
			
			if (wPoint_reached and (self.wPoint_count < waypoints_parm_length - 1)):

				self.wPoint_count = self.wPoint_count + 1 #waypoint increment

				print('waypoint_incremented',self.wPoint_count)

				desired_x = wplist[self.wPoint_count][0]
				desired_y = wplist[self.wPoint_count][1]
				desired_z = wplist[self.wPoint_count][2]

				prese_point =[drone_pose.x,drone_pose.y,drone_pose.z]

				desired_wpoint = [desired_x,desired_y,desired_z]

				self.trag_time = 0

				print('inc',prese_point,desired_wpoint,self.trag_time)

				traj = TrajectoryGenerator(prese_point, desired_wpoint , self.trag_time+0.1 ,[0,0,0],[0,0,0])
				
				traj.solve()

				self.x_c = traj.x_c
				self.y_c = traj.y_c
				self.z_c = traj.z_c

				self.increment_trag_time = 0.1

			elif(wPoint_reached and (self.wPoint_count == waypoints_parm_length - 1)):
				
				return True

		while not rospy.is_shutdown():

			#print('in while',self.x_c,self.increment_trag_time)

			#print('in while',self.x_c,self.y_c,self.z_c)
			print('while time',self.increment_trag_time)

			des_x_pos = self.calculate_position(self.x_c, self.increment_trag_time)
			des_y_pos = self.calculate_position(self.y_c, self.increment_trag_time)
			des_z_pos = self.calculate_position(self.z_c, self.increment_trag_time)

			des_x_vel = self.calculate_velocity(self.x_c, self.increment_trag_time)
			des_y_vel = self.calculate_velocity(self.y_c, self.increment_trag_time)
			des_z_vel = self.calculate_velocity(self.z_c, self.increment_trag_time)

			x_pos = self.drone_position.point.x
			y_pos = self.drone_position.point.y
			z_pos = self.drone_position.point.z

			x_vel = self.drone_velocity.vector.x
			y_vel = self.drone_velocity.vector.y
			z_vel = self.drone_velocity.vector.z

			x_cmd = self.Kp_x * (des_x_pos - x_pos) + self.Kd_x * (des_x_vel - x_vel)
			y_cmd = self.Kp_y * (des_y_pos - y_pos) + self.Kd_y * (des_y_vel - y_vel)
			z_cmd = self.Kp_z * (des_z_pos - z_pos) + self.Kd_z * (des_z_vel - z_vel)

			xf_cmd = round(x_cmd,4)
			yf_cmd = round(y_cmd,4)
			zf_cmd = round(z_cmd,4)

			print('d_cmd',xf_cmd,yf_cmd,zf_cmd)

			vel_wpoint_cmd.axes.append(xf_cmd)
			vel_wpoint_cmd.axes.append(yf_cmd)
			vel_wpoint_cmd.axes.append(zf_cmd)
			vel_wpoint_cmd.axes.append(0) 			

			self.velocity_publisher.publish(vel_wpoint_cmd)

			del vel_wpoint_cmd.axes[:]

			inc = wpoint_increment()
			
			if inc == True:
				break

			self.rate.sleep()  # Publish at the desired rate..            

			self.increment_trag_time = 0.1



	def calculate_position(self,c, t):
		"""
		Calculates a position given a set of quintic coefficients and a time.

		Args
			c: List of coefficients generated by a quintic polynomial 
				trajectory generator.
			t: Time at which to calculate the position

		Returns
			Position
		"""
		return c[0] * t**5 + c[1] * t**4 + c[2] * t**3 + c[3] * t**2 + c[4] * t + c[5]


	def calculate_velocity(self,c, t):
		"""
		Calculates a velocity given a set of quintic coefficients and a time.

		Args
			c: List of coefficients generated by a quintic polynomial 
				trajectory generator.
			t: Time at which to calculate the velocity

		Returns
			Velocity
		"""
		return 5 * c[0] * t**4 + 4 * c[1] * t**3 + 3 * c[2] * t**2 + 2 * c[3] * t + c[4]


	def calculate_acceleration(self,c, t):
		"""
		Calculates an acceleration given a set of quintic coefficients and a time.

		Args
			c: List of coefficients generated by a quintic polynomial 
				trajectory generator.
			t: Time at which to calculate the acceleration

		Returns
			Acceleration
		"""
		return 20 * c[0] * t**3 + 12 * c[1] * t**2 + 6 * c[2] * t + 2 * c[3]


	def rotation_matrix(self,roll, pitch, yaw):
		"""
		Calculates the ZYX rotation matrix.

		Args
			Roll: Angular position about the x-axis in radians.
			Pitch: Angular position about the y-axis in radians.
			Yaw: Angular position about the z-axis in radians.

		Returns
			3x3 rotation matrix as NumPy array
		"""
		return np.array(
			[[cos(yaw) * cos(pitch), -sin(yaw) * cos(roll) + cos(yaw) * sin(pitch) * sin(roll), sin(yaw) * sin(roll) + cos(yaw) * sin(pitch) * cos(roll)],
			 [sin(yaw) * cos(pitch), cos(yaw) * cos(roll) + sin(yaw) * sin(pitch) *
			  sin(roll), -cos(yaw) * sin(roll) + sin(yaw) * sin(pitch) * cos(roll)],
			 [-sin(pitch), cos(pitch) * sin(roll), cos(pitch) * cos(yaw)]
			 ])


	def set_local_ref_service(self):

		set_local = rospy.ServiceProxy('/dji1_sdk/set_local_pos_ref', SetLocalPosRef)
		result_local = set_local()
		
		print("Hi Drone1 :- set_local_reference_Cmd-Sent to drone")

		return result_local


	def sdkActivation(self,sdk_flag):

		if (sdk_flag == True):
			act = rospy.ServiceProxy('/dji1_sdk/sdk_control_authority', SDKControlAuthority)
			result_act = act(1)

		elif(sdk_flag == False):
			act = rospy.ServiceProxy('/dji1_sdk/sdk_control_authority', SDKControlAuthority)
			result_act = act(0)

		# print(result_act.result)
		if result_act.result == True:

			result_activation = "activated"
			print("Hi Drone1 :- SDK-Activation - TRUE")
			return result_activation

		elif result_act.result ==False:

			result_activation = "Deactivated"   
			print("Hi Drone1 :- SDK-Activation - FLASE")
			return result_activation
		

	def takeoff(self):

		#" services to call before giving commands"

		s = rospy.ServiceProxy('/dji1_sdk/drone_task_control', DroneTaskControl)

		result_takeoff = s(4)

		if result_takeoff:
			print("Hi Drone1 :- Takeoff_Cmd- TRUE")
			return True
		else:  
			print("Hi Drone1 :- Takeoff_Cmd- FALSE")
			return False  

		
	def land(self):

		land = rospy.ServiceProxy('/dji1_sdk/drone_task_control', DroneTaskControl)

		result_land = land(6)

		if result_land:
			print("Hi Drone1 :- Landing_Cmd - TRUE")
			return True
		else:  
			print("Hi Drone1 :- Landing_Cmd - FALSE")
			return False  