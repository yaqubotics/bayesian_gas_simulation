#!/usr/bin/env python

#
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import os
import csv
import sklearn
from PIL import Image, ImageOps
import random
from scipy import special
import roslib
import rospy
import sys
import cv2
from std_msgs.msg import String, Float64, Int32
from olfaction_msgs.msg import gas_sensor
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseStamped, Point
from visualization_msgs.msg import Marker,MarkerArray
from move_base_msgs.msg import MoveBaseActionGoal
from nav_msgs.msg import Path
from datetime import datetime
from scipy import stats
from implementation import *

#import keyboard

verbose = True
Gaussian = 1
KDE = 2
delta = 1e-36
def printv(ver,str_):
	if(ver):
		print str_ 

def open_partition_csv_matrix(file_name,delim):
	a_list = []
	with open(file_name) as csv_file:
		csv_reader = csv.reader(csv_file, delimiter=delim)
		for row in csv_reader:
			b_list = []
			if len(row) == 112:
				del row[-1]
			for col in row:
				b_list.append(float(col))
			a_list.append(b_list)
	return np.matrix(a_list)

def read_dataset_kde(sim_location,room_source,time_variant): # 50x13x20
	room_measure = np.arange(1,51,1)
	dict_of_kde_pdf = {}
	for rs in room_source:
		str_room_source = '{:02d}'.format(rs)
		for rm in room_measure:
			str_room_measure = '{:02d}'.format(rm)
			file_name = sim_location+'/dataset/gas_source_'+str_room_source+'/meas_at_'+str_room_measure+'/dataset_source_'+str_room_source+'_meas_'+str_room_measure+'_min_20.csv'
			#print file_name
			with open(file_name) as csv_file:
				csv_reader = csv.reader(csv_file, delimiter=' ')
				#print csv_reader
				#print list(csv_reader)
				time = 1
				for row in csv_reader:
					row = np.array(row, dtype=np.float32)
					#print row
					try:
						dict_of_kde_pdf[(rs,rm,time)] = (1,stats.gaussian_kde(row))
					except:
						dict_of_kde_pdf[(rs,rm,time)] = (0,0)
						if(np.sum(row) > 0):
							print 'WARNING: THERE IS A BUG IN KDE DATASET'
					time = time + 1
	return dict_of_kde_pdf

def read_dataset_kde_pose_only(sim_location,room_source): # 50x13x20
	room_measure = np.arange(1,51,1)
	dict_of_kde_pdf = {}
	for rs in room_source:
		str_room_source = '{:02d}'.format(rs)
		for rm in room_measure:
			str_room_measure = '{:02d}'.format(rm)
			file_name = sim_location+'/dataset/gas_source_'+str_room_source+'/meas_at_'+str_room_measure+'/dataset_source_'+str_room_source+'_meas_'+str_room_measure+'_min_20.csv'
			#print file_name
			with open(file_name) as csv_file:
				csv_reader = csv.reader(csv_file, delimiter=' ')
				#print csv_reader
				#print list(csv_reader)
				row_time_interval = []
				for row in csv_reader:
					row = np.array(row, dtype=np.float32)
					row_time_interval = row_time_interval+list(row)
				try:
					dict_of_kde_pdf[(rs,rm)] = (1,stats.gaussian_kde(row_time_interval))
				except:
					dict_of_kde_pdf[(rs,rm)] = (0,0)
					if(np.sum(row_time_interval) > 0):
						print 'WARNING: THERE IS A BUG IN KDE DATASET'
	return dict_of_kde_pdf

def read_dataset_gaussian(sim_location,room_source,time_variant): # 50x13x20
	room_measure = np.arange(1,51,1)
	dict_of_gaussian_pdf = {}
	for rs in room_source:
		str_room_source = '{:02d}'.format(rs)
		for rm in room_measure:
			str_room_measure = '{:02d}'.format(rm)
			file_name = sim_location+'/dataset/gas_source_'+str_room_source+'/meas_at_'+str_room_measure+'/dataset_source_'+str_room_source+'_meas_'+str_room_measure+'_min_20.csv'
			#print file_name
			with open(file_name) as csv_file:
				csv_reader = csv.reader(csv_file, delimiter=' ')
				#print csv_reader
				#print list(csv_reader)
				time = 1
				for row in csv_reader:
					row = np.array(row, dtype=np.float32)
					#print row
					if(np.count_nonzero(row)>0):
						dict_of_gaussian_pdf[(rs,rm,time)] = (1,(np.mean(row),np.std(row)))
					else:
						dict_of_gaussian_pdf[(rs,rm,time)] = (0,0)
					time = time + 1

	return dict_of_gaussian_pdf

def gaussian_function(x,mean,std):
	if(std == 0):
		std = 0.000001
	return (1.0/(std*np.sqrt(2*np.pi)))*np.exp(-0.5*(((x-mean)/std)**2))#+(random.random())*0.001

def open_partition_csv(file_name,delim):
	a_list = []
	with open(file_name) as csv_file:
		csv_reader = csv.reader(csv_file, delimiter=delim)
		for row in csv_reader:
			b_list = []
			if len(row) == 112:
				del row[-1]
			for col in row:
				b_list.append(float(col))
			a_list.append(b_list)
	return np.matrix(a_list)

def calculate_likelihood_gaussian(concentration,robot_at_room,source_room,time_variant,dict_of_gaussian_pdf):
	specific_pdf = dict_of_gaussian_pdf[(source_room,robot_at_room,time_variant)]
	if(specific_pdf[0] == 1):
		the_likelihood = gaussian_function(concentration,specific_pdf[1][0],specific_pdf[1][1])
		if(the_likelihood > 1):
			the_likelihood = 1.0
	else:
		if(concentration < 1.0):
			the_likelihood = 1.0 #0.000001
		else:
			the_likelihood = delta
	return the_likelihood

def calculate_likelihood_kde(concentration,robot_at_room,source_room,time_variant,dict_of_kde_pdf):
	specific_pdf = dict_of_kde_pdf[(source_room,robot_at_room,time_variant)]
	#if(source_room == 13) and (robot_at_room == 13) and (time_variant == 15):
	if(specific_pdf[0] == 1):
		the_likelihood = specific_pdf[1].evaluate(concentration)[0]
		if(the_likelihood > 1): # sense small ppm in sparse room
			the_likelihood = 1.0
		#else:
		#	the_likelihood = 1e-40 #0.000001
	else:
		if(concentration < 1):
			the_likelihood = 1.0
		else:
			the_likelihood = delta #0.000001
	return the_likelihood

def calculate_likelihood_kde_pose_only(concentration,robot_at_room,source_room,dict_of_kde_pdf):
	specific_pdf = dict_of_kde_pdf[(source_room,robot_at_room)]
	#if(source_room == 13) and (robot_at_room == 13) and (time_variant == 15):
	if(specific_pdf[0] == 1):
		the_likelihood = specific_pdf[1].evaluate(concentration)[0]
		if(the_likelihood > 1): # sense small ppm in sparse room
			the_likelihood = 1.0
		#else:
		#	the_likelihood = 1e-40 #0.000001
	else:
		if(concentration < 1):
			the_likelihood = 1.0
		else:
			the_likelihood = delta #0.000001
	return the_likelihood

def calculate_likelihood_kde2(concentration,robot_at_room,source_room,time_variant,file_location,Partition):
	start = rospy.get_time()
	A4 = np.ones(Partition.shape)*robot_at_room
	room_boolean = Partition == A4
	filenames= os.listdir (file_location)
	result = []
	for filename in filenames: # loop through all the files and folders
		if os.path.isdir(os.path.join(os.path.abspath(file_location), filename)): # check whether the current object is a folder or not
			result.append(filename)
	sub_result = []
	str_of_source_room = '{:02d}'.format(source_room)
	for r in result:
		if("_r_"+str_of_source_room in r):
			sub_result.append(r)

	the_data = []
	for sr in sub_result:
		str_sec = '{:4d}'.format(time_variant)
		the_matrix_ori = open_partition_csv(file_location+sr+'/PDF_gasType_10_iter_'+str_sec+'.csv',' ')
		#the_matrix = np.multiply(the_matrix_ori,room_boolean)
		A=the_matrix_ori+room_boolean
		B=A.tolist()
		C=np.argwhere(B)
		the_data = [x-1 for x in C]
		#for p in range(the_matrix_ori.shape[0]):
		#	for q in range(the_matrix_ori.shape[1]):
		#		if(room_boolean[p,q] == 1):
		#			the_data.append(the_matrix_ori[p,q])
	print B
	print "get the data spent time: ", rospy.get_time() - start
	try:
		kde_obj = stats.gaussian_kde(the_data)
		the_likelihood = kde_obj.evaluate(concentration)[0]
	except:
		the_likelihood = 0.00001
	print "calculate likelihood time spent: ",rospy.get_time() - start
	return the_likelihood

def normalize(a):
	s = []
	for i in a:
		try:
			norm = float(i)/sum(a)
			s.append(norm)
		except:
			print "WARNING normalization"
			print i, float(i), sum(a)
			norm = float(i)/sum(a)
			s.append(norm) 
	return s

def gas_meas_cb(msg_var):
	global gas_meas
	gas_meas = msg_var.raw

def pose_cb(msg_var):
	global robot_pose
	robot_pose = msg_var.pose.pose

def iteration_truth_cb(msg_var):
	global iteration_truth
	iteration_truth = msg_var.data

def euclidean_distance(a,b):
	return np.sqrt((a[0]-b[0])**2+(a[1]-b[1])**2)

def main(args):

	global gas_meas, robot_pose, iteration_truth
	refresh_posterior_after_source_detected = False
	use_posterior_pose_only = False
	goal_is_the_largest_accumulated_time = True
	iteration_truth = 0
	time_gas_started = 360
	final_goal_room = 15
	room_source = list(np.arange(1,51,1)) #[1,2,3,4,5,6,7,8,9,10,11,12,13]
	if(use_posterior_pose_only):
		time_variant = list(np.arange(1,2,1))
	else:
		time_variant = list(np.arange(1,21,1)) #minute 1-20 every 1 minute
	#source_at_room = (12,45.5,3.6)
	#source_at_room = (13,48.5,3.3)
	#source_at_room = (28,22.0,11.5)
	source_at_room = (50,26.0,21.0)
	time_interval = 60

	gas_meas = 0.0
	robot_pose = Pose()
	robot_ID = '1'
	rospy.init_node("robot_"+robot_ID)
	r=rospy.Rate(10)
	print "PROGRAM START"
	global_param = rospy.get_param('~global_param_name','default')
	simulation_folder = rospy.get_param('~simulation_folder','default')
	gas_source_folder = rospy.get_param('~gas_source_folder','default')
	measurement_time_sampling = float(rospy.get_param('~measurement_time_sampling','1.0'))
	plot_time_sampling = measurement_time_sampling*4
	min_sensor_time_sampling = 1.0
	print "simulation_folder: ",simulation_folder
	start_measurement=rospy.get_time()

	# subscriber declaration
	gas_meas_sub = rospy.Subscriber("/PID/Sensor_reading",gas_sensor,queue_size = 10,callback=gas_meas_cb)
	pose_sub = rospy.Subscriber("/odom",Odometry,queue_size = 10,callback=pose_cb)
	iteration_truth_sub = rospy.Subscriber("/iteration_truth",Int32,queue_size = 10,callback=iteration_truth_cb)

	goal_pub = rospy.Publisher("/move_base/goal",MoveBaseActionGoal,queue_size=10)
	path_pub = rospy.Publisher("/path",Path,queue_size=10)

	
	
	
	file_name = simulation_folder+'/OccupancyGrid3Droom.csv'
	with open(file_name) as csv_file:
		csv_reader = csv.reader(csv_file, delimiter=',')
		room_matrix = list(csv_reader)

	file_name = simulation_folder+'/door_room.csv'
	with open(file_name) as csv_file:
		csv_reader = csv.reader(csv_file, delimiter=',')
		door_room_source  = list(csv_reader)

	file_name = simulation_folder+'/centroid_room.csv'
	with open(file_name) as csv_file:
		csv_reader = csv.reader(csv_file, delimiter=' ')
		centroid_room_source  = list(csv_reader)

	file_name = simulation_folder+'/outlet_room.csv'
	with open(file_name) as csv_file:
		csv_reader = csv.reader(csv_file, delimiter=' ')
		outlet_room_source  = list(csv_reader)

	file_name = simulation_folder+'/graph.csv'
	with open(file_name) as csv_file:
		csv_reader = csv.reader(csv_file, delimiter=' ')
		graph_room = list(csv_reader)

	env_min_x=-0.0004
	env_min_y=-0.1456
	environment_cell_size = 0.2
	idx = 0

	posterior = []
	list_max_posterior_room = []
	for i in range(len(room_source)*len(time_variant)):
		posterior.append(1.0/(len(room_source)*len(time_variant)))
	list_posterior_room_time = []
	list_posterior_room = []
	list_likelihood = []
	for i in range(len(room_source)*len(time_variant)):
		list_posterior_room_time.append([posterior[i]])
		list_likelihood.append(0)

	for i in range(len(room_source)):
		list_posterior_room.append([1.0/(len(room_source))])

	goal_marker_topic = 'goal_marker'
	goal_marker_pub = rospy.Publisher(goal_marker_topic, Marker,queue_size=10)
	goal_marker = Marker()
	goal_marker.header.frame_id = "/map"
	goal_marker.header.stamp=rospy.Time.now()
	goal_marker.type = goal_marker.CUBE
	goal_marker.action = goal_marker.ADD
	goal_marker.scale.x = 0.5
	goal_marker.scale.y = 0.5
	goal_marker.scale.z = 0.5
	goal_marker.pose.orientation.w = 1.0
	goal_marker.color.r =255.0/255.0
	goal_marker.color.g= 255.0/255.0
	goal_marker.color.b =255.0/255.0
	goal_marker.color.a =1.0

	list_size_room = []
	for i in room_source:
		coord_list_x = []
		coord_list_y = []
		for m in range(len(room_matrix)):
			for n in range(len(room_matrix[0])):
				if (int(room_matrix[m][n]) == i):
					coord_list_x.append(m*0.2+env_min_x) 
					coord_list_y.append(n*0.2+env_min_y)
		list_size_room.append((np.max(coord_list_x)+0.2,np.min(coord_list_x),np.max(coord_list_y)+0.2,np.min(coord_list_y)))
	
	room_marker_array_topic = 'room_marker'
	room_marker_array_pub = rospy.Publisher(room_marker_array_topic, MarkerArray,queue_size=10)
	room_marker_array = MarkerArray()
	room_marker_array.markers = []
	for i in room_source:
		room_marker = Marker()
		room_marker.header.frame_id = "/map"
		room_marker.header.stamp=rospy.Time.now()
		room_marker.id = i
		room_marker.type = room_marker.CUBE
		room_marker.action = room_marker.ADD
		room_marker.scale.x = list_size_room[i-1][0]-list_size_room[i-1][1]
		room_marker.scale.y = list_size_room[i-1][2]-list_size_room[i-1][3]
		room_marker.scale.z = 3
		room_marker.color.r =255.0/255.0
		room_marker.color.g= 55.0/255.0
		room_marker.color.b =255.0/255.0
		room_marker.color.a =0.85
		#room_marker.lifetime = rospy.Duration(0)
		room_marker.pose.position.x = (list_size_room[i-1][0]+list_size_room[i-1][1])/2.0
		room_marker.pose.position.y = (list_size_room[i-1][2]+list_size_room[i-1][3])/2.0
		room_marker.pose.position.z = 1.5
		room_marker.pose.orientation.x = 0.0
		room_marker.pose.orientation.y = 0.0
		room_marker.pose.orientation.z = 0.0
		room_marker.pose.orientation.w = 1.0
		room_marker_array.markers.append(room_marker)
	
	if(not use_posterior_pose_only):
		time_marker_array_topic = 'time_marker'
		time_marker_array_pub = rospy.Publisher(time_marker_array_topic, MarkerArray,queue_size=10)
		time_marker_array = MarkerArray()
		time_marker_array.markers = []
	
		#for i in room_source:
		for j in time_variant:
			time_marker = Marker()
			time_marker.header.frame_id = "/map"
			time_marker.header.stamp=rospy.Time.now()
			time_marker.id = (i-1)*len(time_variant)+(j-1)
			time_marker.type = room_marker.CUBE
			time_marker.action = room_marker.ADD
			time_marker.scale.x = list_size_room[i-1][0]-list_size_room[i-1][1]
			time_marker.scale.y = (list_size_room[i-1][2]-list_size_room[i-1][3])/len(time_variant)
			time_marker.scale.z = 3
			time_marker.color.r =255.0/255.0
			time_marker.color.g= 0.0/255.0
			time_marker.color.b =0.0/255.0
			time_marker.color.a =0.85
			y1 = list_size_room[source_at_room[0]-1][2]+(list_size_room[source_at_room[0]-1][3]-list_size_room[source_at_room[0]-1][2])*(j-1)/len(time_variant)
			y2 = list_size_room[source_at_room[0]-1][2]+(list_size_room[source_at_room[0]-1][3]-list_size_room[source_at_room[0]-1][2])*(j)/len(time_variant)
			
			time_marker.pose.position.x = (list_size_room[source_at_room[0]-1][0]+list_size_room[source_at_room[0]-1][1])/2.0
			time_marker.pose.position.y = (y1+y2)/2.0
			time_marker.pose.position.z = 1.5
			time_marker.pose.orientation.x = 0.0
			time_marker.pose.orientation.y = 0.0
			time_marker.pose.orientation.z = 0.0
			time_marker.pose.orientation.w = 1.0
			time_marker_array.markers.append(time_marker)
	
	path_display = Path()
	path_display.header.frame_id = "/map"
	path_display.header.stamp = rospy.Time.now()
	path_display.poses = [] 
	
	#print room_marker_array
	robot_detect_gas = False
	max_room_source = 22
	max_post_room_accumulated_idx = max_room_source-1
	prev_goal = 0
	goal_treshold = 1.0
	concentration_data = []

	room_partition_matrix = open_partition_csv_matrix(simulation_folder+'/OccupancyGrid3Droom.csv',',')

	start_read_dataset=rospy.get_time()
	likelihood_function_used = KDE
	'''
	if(use_posterior_pose_only):
			dict_of_kde_pdf=read_dataset_kde_pose_only(simulation_folder,room_source)
	else:
		if(likelihood_function_used == KDE):
			dict_of_kde_pdf=read_dataset_kde(simulation_folder,room_source,time_variant)
		
		if(likelihood_function_used == Gaussian):
			dict_of_gaussian_pdf=read_dataset_gaussian(simulation_folder,room_source,time_variant)
	'''
	print 'read dataset spent:', rospy.get_time()-start_read_dataset

	start_plot=rospy.get_time()
	start_sensor_reading = rospy.get_time()
	while(iteration_truth == 0):
		pass
	start_enter_the_building = rospy.get_time()
	list_time_secs = [iteration_truth]
	mission_completed = False
	posterior_refresh = False
	time_estimation_error_list = []
	is_in_source_room_list = []
	# the prob. the gas leaks 15 minutes before the robots enter:
	time_variant_flag = time_variant[:]
	current_goal_x = float(centroid_room_source[final_goal_room-1][0])
	current_goal_y = float(centroid_room_source[final_goal_room-1][1])
	max_concentration = 0.0
	list_room_not_visited = room_source[:]
	current_state = 1
	concentration_data = []
	grrm = []
	diagram4 = GridWithWeights(len(room_matrix), len(room_matrix[0]))
	diagram4.walls = [] # [(0, 7), (0, 8),(1, 7), (1, 8), (2, 7), (2, 8), (3, 7), (3, 8), (3, 5), (3, 6),(3, 3), (3, 4)]
	for i in range(len(room_matrix)):
		for j in range(len(room_matrix[0])):
			if int(room_matrix[i][j]) == 0:
				diagram4.walls.append((i,j))

	trajectory_length = 0
	prev_robot_pose = Pose()
	prev_robot_pose.position.x = robot_pose.position.x
	prev_robot_pose.position.y = robot_pose.position.y
	source_found = False
	high_cont_room_list = []
	max_concentration_room_list = []
	for i in room_source:
		max_concentration_room_list.append(0)
	while not rospy.is_shutdown():
		# get measurement as many as the sensor can and average it
		if gas_meas > 0:
			concentration_data.append(gas_meas)
		if(rospy.get_time()-start_sensor_reading > min_sensor_time_sampling):
			start_sensor_reading = rospy.get_time()
			x1=robot_pose.position.x
			y1=robot_pose.position.y
			x2=prev_robot_pose.position.x
			y2=prev_robot_pose.position.y
			current_prev_distance = euclidean_distance((x1,y1),(x2,y2))
			prev_robot_pose.position.x = robot_pose.position.x
			prev_robot_pose.position.y = robot_pose.position.y
			trajectory_length=trajectory_length+current_prev_distance
			
			pose_temp = PoseStamped()
			pose_temp.pose.position.x = robot_pose.position.x
			pose_temp.pose.position.y = robot_pose.position.y
			pose_temp.pose.position.z = 0.2
			path_display.poses.append(pose_temp)
			path_pub.publish(path_display)

		if(rospy.get_time()-start_measurement > measurement_time_sampling):
			start_measurement=rospy.get_time()
			robot_time = iteration_truth-time_gas_started
			if (iteration_truth-time_gas_started > time_interval*time_variant_flag[0]) and (not use_posterior_pose_only):
				print "========================================"
				print "++++++++++ SHIFT TIME ++++++++++++++++++"
				print "========================================"
				del time_variant_flag[0] 
				time_variant = time_variant[1:]+[None]
				#for rs in room_source:
				#	posterior[(rs-1):(rs-1)+len(time_variant)] = posterior[(rs-1)+1:(rs-1)+len(time_variant)] + [0]
			

			time_now_secs = iteration_truth
			print "Iteration truth: ",iteration_truth
			print "Trajectory length:",trajectory_length,"meter"
			#concentration = np.mean(concentration_data)
			concentration = gas_meas
			
			grid_pos_x = int(np.ceil((robot_pose.position.x-env_min_x)/environment_cell_size))
			grid_pos_y = int(np.ceil((robot_pose.position.y-env_min_y)/environment_cell_size))
			robot_at_room = int(room_matrix[grid_pos_x][grid_pos_y])
			if(robot_at_room == 0):
				grid_pos_x = int(np.ceil((robot_pose.position.x-env_min_x)/environment_cell_size))
				grid_pos_y = int(np.floor((robot_pose.position.y-env_min_y)/environment_cell_size))
				robot_at_room = int(room_matrix[grid_pos_x][grid_pos_y])
				if(robot_at_room == 0):
					grid_pos_x = int(np.floor((robot_pose.position.x-env_min_x)/environment_cell_size))
					grid_pos_y = int(np.ceil((robot_pose.position.y-env_min_y)/environment_cell_size))
					robot_at_room = int(room_matrix[grid_pos_x][grid_pos_y])
					if(robot_at_room == 0):
						grid_pos_x = int(np.floor((robot_pose.position.x-env_min_x)/environment_cell_size))
						grid_pos_y = int(np.floor((robot_pose.position.y-env_min_y)/environment_cell_size))
						robot_at_room = int(room_matrix[grid_pos_x][grid_pos_y])
			if(concentration > max_concentration_room_list[robot_at_room-1]):
				max_concentration_room_list[robot_at_room-1] = concentration
			robot_detect_gas = True
			#if concentration > 0:
			#concentration_data.append(concentration)
			print "current state",current_state
			if(current_state == 1):
				current_goal_x = float(door_room_source[final_goal_room-1][0])
				current_goal_y = float(door_room_source[final_goal_room-1][1])
				if(len(high_cont_room_list)>0):
					max_concentration_room_list_temp = max_concentration_room_list[:]
					print "not visited:",list_room_not_visited
					for i in range(len(max_concentration_room_list)):
						#current_max_room = high_cont_room_list[-1][0]
						if(i+1 not in list_room_not_visited):
							print "room:",i+1
							visited_room = 0
							for gr in graph_room[i]:
								print "neighbor",gr
								if not (int(gr) in list_room_not_visited):
									print "already visited"
									visited_room = visited_room+1
									
							if(visited_room == len(graph_room[i])):
								max_concentration_room_list_temp[i] = -1
								print "all visited room",i+1
					print max_concentration_room_list_temp
					idx_max = np.argmax(max_concentration_room_list_temp)
					print "room to explore:",idx_max+1
					for gr in graph_room[idx_max]:
						if(int(gr) in list_room_not_visited) and (int(gr) not in grrm):
							grrm.insert(0,int(gr))
							print "room neighbor-",int(gr)
					if(len(grrm) > 0):
						current_goal_x = float(centroid_room_source[grrm[0]-1][0])
						current_goal_y = float(centroid_room_source[grrm[0]-1][1])
						current_state = 2

			if(concentration > max_concentration):
				print "Higher concentration detected",concentration
				max_concentration = concentration #np.mean(concentration_data)
				concentration_data = []
				high_cont_room_list.append((robot_at_room,max_concentration))
				#list 2 rooms near the robot room
				'''
				min_distance = 999999
				min_room = 0
				
				for i in list_room_not_visited:
					x1 = float(door_room_source[i-1][0])
					y1 = float(door_room_source[i-1][1])
					x2 = float(door_room_source[robot_at_room-1][0])
					y2 = float(door_room_source[robot_at_room-1][1])
					dist = euclidean_distance((x1,y1),(x2,y2))
					#start, goal = (x1, y1), (x2, y2)
					#came_from, cost_so_far,path = theta_star_search2(diagram4, start, goal)
					#came_from, cost_so_far = a_star_search(diagram4, start, goal)
					#path=reconstruct_path(came_from, start=start, goal=goal)
					if min_distance > dist:#cost_so_far[goal]:
						min_distance = dist #cost_so_far[goal]
						min_room = i
				'''
				print "Robot at room",robot_at_room
				#grrm = []
				for gr in graph_room[robot_at_room-1]:
					if(int(gr) in list_room_not_visited) and (int(gr) not in grrm):
						grrm.insert(0,int(gr))
						print "room neighbor-",int(gr)
				if(len(grrm) > 0):
					current_goal_x = float(centroid_room_source[grrm[0]-1][0])
					current_goal_y = float(centroid_room_source[grrm[0]-1][1])
					current_state = 2
			if(current_state == 2):
				x1 = robot_pose.position.x
				y1 = robot_pose.position.y
				x2 = current_goal_x
				y2 = current_goal_y
				if(euclidean_distance((x1,y1),(x2,y2)) < 1.0):
					if(robot_at_room in grrm):
						grrm.remove(robot_at_room)
						print "Remove room-",robot_at_room,"from list"
					if(len(grrm) > 0):
						current_goal_x = float(centroid_room_source[grrm[0]-1][0])
						current_goal_y = float(centroid_room_source[grrm[0]-1][1])
					else:
						current_state = 1
					if(robot_at_room == source_at_room[0]):
						print "SOURCE FOUND!!"
						source_found = True
					else:
						print "NO SOURCE :(( at",robot_at_room
						#max_concentration = max_concentration/2

			if(robot_at_room in list_room_not_visited):
				print "Remove room-",robot_at_room
				list_room_not_visited.remove(robot_at_room)

			'''
			printv(verbose,'conc: '+str(gas_meas))
			#in which room the measurement is done?
			#print "pose:",robot_pose.position.x,robot_pose.position.y
			
			printv(verbose,'room:'+str(robot_at_room)+' '+str(grid_pos_x)+' '+str(grid_pos_y))
			str_of_room = '{:02d}'.format(robot_at_room)
			#get the mean and std in room at time 1-15 min
			pos_idx = 0

			list_likelihood = []
			for rs in room_source:
				#str_of_rs = '{:02d}'.format(rs)
				#find file
				#all_file = []
				#the_list_dir_path = simulation_folder+'/analysis/gas_source_'+str_of_rs+'/meas_at_'+str_of_room+'/' 
				#filenames= os.listdir (the_list_dir_path)
				
				#for file in filenames:
				#	if("mean_std_min" in file):
				#		mean_std_csv_file = file
				#file_name = the_list_dir_path+mean_std_csv_file
				#list_mean_time = []
				#list_std_time = []
				#list_post_time = []
				#with open(file_name) as csv_file:
				#	csv_reader = csv.reader(csv_file, delimiter=' ')
				#	csv_reader = list(csv_reader)
				if(use_posterior_pose_only):
					likelihood = calculate_likelihood_kde_pose_only(concentration,robot_at_room,rs,dict_of_kde_pdf)
					list_likelihood.append(likelihood)
					posterior[pos_idx] = likelihood*posterior[pos_idx]
					pos_idx = pos_idx+1	
				else:
					for tv in time_variant:
						if(likelihood_function_used == KDE):
							if(tv == None):
								likelihood = 1e-7
							else:
								#if(tv+1 == None) or (tv+1 == len(time_variant)+1):
								likelihood=calculate_likelihood_kde(concentration,robot_at_room,rs,tv,dict_of_kde_pdf)
								#else:
								#	likelihood_1=calculate_likelihood_kde(concentration,robot_at_room,rs,tv,dict_of_kde_pdf)
								#	likelihood_2=calculate_likelihood_kde(concentration,robot_at_room,rs,tv+1,dict_of_kde_pdf)
								#	likelihood_1 = likelihood_1*(time_interval-(robot_time % time_interval))/time_interval
								#	likelihood_2 = likelihood_2*(robot_time % time_interval)/time_interval 
								#	likelihood = likelihood_1+likelihood_2
							if(likelihood_function_used == Gaussian):
								if(tv == None):
									likelihood = 1e-7
								else:
									likelihood=calculate_likelihood_gaussian(concentration,robot_at_room,rs,tv,dict_of_gaussian_pdf)
							if((rs == source_at_room[0]) and ((tv == 1) or (tv == 17))):
								print 'prior at minute-',tv,'is:',posterior[pos_idx]
								print 'likelihood at minute-',tv,'is:',likelihood
							posterior[pos_idx] = likelihood*posterior[pos_idx]
							list_likelihood.append(likelihood)
						pos_idx = pos_idx+1	
			posterior=normalize(posterior)
			max_room_accumulated = []
			for i in range(len(room_source)):
				temp_list = []
				for j in range(len(time_variant)):
					temp_list.append(posterior[i*len(time_variant)+j])
				room_marker_array.markers[i].scale.z = 3*np.sum(temp_list)
				room_marker_array.markers[i].pose.position.z = room_marker_array.markers[i].scale.z/2.0
				max_room_accumulated.append(np.sum(temp_list))
				list_posterior_room[i].append(np.sum(temp_list))
			
			room_marker_array_pub.publish(room_marker_array)
			
			max_post_room_accumulated_idx = np.argmax(max_room_accumulated)
			#print 'robot room:',robot_at_room
			#print 'robot pose',(robot_pose.position.x,robot_pose.position.y)
			#print 'robot at room:',float(door_room_source [robot_at_room-1][0]),float(door_room_source [robot_at_room-1][1])
			#print "dist to center:", euclidean_distance((robot_pose.position.x,robot_pose.position.y),(float(door_room_source [robot_at_room-1][0]),float(door_room_source [robot_at_room-1][1])))
			#print 'goal', float(door_room_source [max_room_source-1][0]),float(door_room_source [max_room_source-1][1])
			#print 'goal', float(door_room_source [max_post_room_accumulated_idx][0]),float(door_room_source [max_post_room_accumulated_idx][1])
			if(robot_at_room in room_source) and (robot_at_room != source_at_room[0]) and (euclidean_distance((robot_pose.position.x,robot_pose.position.y),(float(door_room_source [robot_at_room-1][0]),float(door_room_source [robot_at_room-1][1]))) < goal_treshold):
				print "Force posterior at room-",robot_at_room,"to be zero"
				for j in range(len(time_variant)):
					posterior[(robot_at_room-1)*len(time_variant)+j] = 0.0
			
			max_post_idx = np.argmax(posterior)
			max_post_room_source = room_source[max_post_idx / len(time_variant)]
			max_post_time_source = max_post_idx % len(time_variant) + 1
			list_max_posterior_room.append(room_source[max_post_idx / len(time_variant)])
			
			max_likelihood_idx = np.argmax(list_likelihood)
			max_likelihood_room_source = room_source[max_likelihood_idx / len(time_variant)]
			max_likelihood_time_source = max_likelihood_idx % len(time_variant) + 1
			print "largest posterior in room-",max_post_room_source,"at minute-",max_post_time_source,"is:",posterior[max_post_idx]
			time_estimation_error_list.append(iteration_truth/time_interval - max_post_time_source)
			if(max_post_room_source == source_at_room[0]):
				is_in_source_room_list.append(1)
			else:
				is_in_source_room_list.append(0)

			print "largest likelihood in room-",max_likelihood_room_source,"at minute-",max_likelihood_time_source,"is:",list_likelihood[max_likelihood_idx]
			
			if(refresh_posterior_after_source_detected):
				if(mission_completed and not posterior_refresh):
					posterior_refresh = True
					for j in range(len(time_variant)):
						posterior[(source_at_room[0]-1)*len(time_variant)+j] = 1.0/len(room_source)
			

			#if(robot_at_room == source_at_room[0]):
			#	if(posterior[max_post_idx] > 0.9):
			#		posterior[max_post_idx] = 0.9
			# publish to that room
			'''
			if(not source_found):
				goal = MoveBaseActionGoal()
				#goal.header.seq = 
				
				goal.goal.target_pose.header.stamp = rospy.Time.now()
				goal.goal.target_pose.header.frame_id = "/map" 
				#goal.goal.target_pose.pose.position.x = float(door_room_source [max_room_source-1][0])
				#goal.goal.target_pose.pose.position.y = float(door_room_source [max_room_source-1][1])
				
				goal.goal.target_pose.pose.position.x = current_goal_x
				goal.goal.target_pose.pose.position.y = current_goal_y
				goal.goal.target_pose.pose.position.z = 0.0
				goal.goal.target_pose.pose.orientation.x = 0.0
				goal.goal.target_pose.pose.orientation.y = 0.0
				goal.goal.target_pose.pose.orientation.z = 0.0
				goal.goal.target_pose.pose.orientation.w = 1.0
	
				goal_pub.publish(goal)
			'''
			goal_marker.header.stamp=rospy.Time.now()
			goal_marker.pose.position.x = goal.goal.target_pose.pose.position.x
			goal_marker.pose.position.y = goal.goal.target_pose.pose.position.y
			goal_marker.pose.position.z = 0.1 
			goal_marker_pub.publish(goal_marker)
			if(euclidean_distance((robot_pose.position.x,robot_pose.position.y),(float(centroid_room_source [source_at_room[0]-1][0]),float(centroid_room_source [source_at_room[0]-1][1]))) < goal_treshold):
				print "Robot arrive at the source point"
				mission_completed = True
			
			if(not mission_completed):
				goal_pub.publish(goal)
			room_marker.header.stamp=rospy.Time.now()

			if(not use_posterior_pose_only):
				for j in range(len(time_variant)):
					time_marker_array.markers[j].scale.z = 3*posterior[(source_at_room[0]-1)*len(time_variant)+j]
					time_marker_array.markers[j].pose.position.z = 3+(time_marker_array.markers[j].scale.z)/2.0
				time_marker_array_pub.publish(time_marker_array)

			for j in range(len(room_source)*len(time_variant)):
				list_posterior_room_time[j].append(posterior[j])
			list_time_secs.append(time_now_secs)
			idx=idx+1
			'''
		if(rospy.get_time()-start_plot > plot_time_sampling):
			'''
			start_plot=rospy.get_time()
			date_time_now = datetime.now()

			fig, ax = plt.subplots()
			for i in range(len(room_source)):
				ax.plot(list_time_secs,list_posterior_room[i])
			plot_title = 'Probability source in room '+str(source_at_room[0])+' ('+str(source_at_room[1])+','+str(source_at_room[2])+') tf:'+str(iteration_truth)
			plt.title(plot_title)
			plt.xlabel('sec')
			plt.ylabel('P')
			legend_p = list(room_source)
			for lp in range(len(legend_p)):
				legend_p[lp]='Room '+str(legend_p[lp])

			plt.legend(legend_p)
			plt.savefig(simulation_folder+'/plot/'+plot_title+' '+str(date_time_now)+'.png')
			plt.close()
			
			fig, ax = plt.subplots()
			for i in range(len(time_variant)):
				if(i == 0) or (i == 2) or (i == 4) or (i == 6) or (i == 8) or (i == 10) or (i == 12) or (i == 14): 
					ax.plot(list_time_secs[10:],list_posterior_room_time[(source_at_room[0]-1)*len(time_variant)+i][10:],linestyle = '-')
				if(i == 15) or (i == 16)  or (i == 17) or (i == 18) or (i == 19): 
					ax.plot(list_time_secs[10:],list_posterior_room_time[(source_at_room[0]-1)*len(time_variant)+i][10:],linestyle = '--')
			plot_title = 'Probability source in room and time '+str(source_at_room[0])+' ('+str(source_at_room[1])+','+str(source_at_room[2])+') tf:'+str(iteration_truth)
			plt.title(plot_title)
			plt.xlabel('sec')
			plt.ylabel('P')
			legend_p = [1,3,5,7,9,11,13,15,16,17,18,19,20] #list(time_variant)
			for lp in range(len(legend_p)):
				legend_p[lp]='Minute '+str(legend_p[lp])

			plt.legend(legend_p)
			plt.savefig(simulation_folder+'/plot/'+plot_title+' '+str(date_time_now)+'.png')
			plt.close()
			
			fig, ax = plt.subplots()
			ax.plot(list_time_secs[1:],time_estimation_error_list)
			ax.plot(list_time_secs[1:],is_in_source_room_list )
			plot_title = 'Time estimation error source in room'+str(source_at_room[0])+' ('+str(source_at_room[1])+','+str(source_at_room[2])+') tf:'+str(iteration_truth)
			plt.title(plot_title)
			plt.xlabel('sec')
			plt.ylabel('Error (sec)')
			plt.legend(['error','flag'])
			plt.savefig(simulation_folder+'/plot/'+plot_title+' '+str(date_time_now)+'.png')
			plt.close()
			'''
			pass
			#plt.plot(list_time_secs,list_posterior_room)
			
			#list_posterior_room_time
		r.sleep()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
	print "Plot here"
	cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)
