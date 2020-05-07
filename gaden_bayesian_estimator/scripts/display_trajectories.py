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
import pickle
from dijkstra_graph import *
from dijkstra import *
#import keyboard

verbose = True
Gaussian = 1
KDE = 2
delta = 1e-6
#0.25 muter2 di awal
#1 panjang
# 3 seems ok
alpha = 0.2
epsilon = 1
visual_check = False
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
			file_name = sim_location+'/dataset_long/gas_source_'+str_room_source+'/meas_at_'+str_room_measure+'/dataset_source_'+str_room_source+'_meas_'+str_room_measure+'_min_20.csv'
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

def read_dataset_kde_specific_time(sim_location,room_source,time_interval): # 50x13x20
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
				row_total = []
				
				for row in csv_reader:
					if(time == 1):
						for r in row:
							row_total.append(0.0)
					row = np.array(row, dtype=np.float32)
					row_total = np.add(row_total,row)
					#row = np.array(row, dtype=np.float32)
					#print row
					if(time % int(time_interval) == 1):
						#print time,
						row_total = row_total/float(time_interval) #np.array(row_total, dtype=np.float32)
						try:
							dict_of_kde_pdf[(rs,rm,time)] = (1,stats.gaussian_kde(row_total))
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
		if(the_likelihood > 1): # 
			the_likelihood = 1.0
		#else:
		#	the_likelihood = 1e-40 #0.000001
	else:
		if(concentration < epsilon):
			the_likelihood = 1.0
		else:
			the_likelihood = delta #0.000001
	if(the_likelihood < delta):
		#print "WARNING likelihood zero"
		the_likelihood = delta
	return the_likelihood

def calculate_likelihood_kde_pose_only(concentration,robot_at_room,source_room,dict_of_kde_pdf):
	specific_pdf = dict_of_kde_pdf[(source_room,robot_at_room)]
	#if(source_room == 13) and (robot_at_room == 13) and (time_variant == 15):
	if(specific_pdf[0] == 1):
		the_likelihood = specific_pdf[1].evaluate(concentration)[0]
		if(the_likelihood > 1): # 
			the_likelihood = 1.0
		#else:
		#	the_likelihood = 1e-40 #0.000001
	else:
		if(concentration < 1):
			the_likelihood = 1.0
		else:
			the_likelihood = delta #0.000001
	if(the_likelihood < delta):
		#print "WARNING likelihood zero"
		the_likelihood = delta
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
		norm = float(i)/sum(a)
		s.append(norm)
	#print a,sum(a)
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

def write_result_to_csv_file(list_robot_pose,list_likelihood_all_time,list_posterior_all_time,list_weighted_posterior_all_time,gas_source_folder):
	dt = str(datetime.now())
	dirName = gas_source_folder+'/pdf_mesh/'+dt
	try:
		# Create target Directory
		os.makedirs(dirName)
		print("Directory " , dirName ,  " Created ") 
	except:
		print("Directory " , dirName ,  " already exists")

	with open(dirName+'/likelihood_mesh_'+dt+'.csv', 'w') as file:
		writer = csv.writer(file,delimiter=' ')
		for item in list_likelihood_all_time:
			writer.writerow([item[0]]+item[1])

	with open(dirName+'/posterior_mesh_'+dt+'.csv', 'w') as file:
		writer = csv.writer(file,delimiter=' ')
		for item in list_posterior_all_time:
			print item
			writer.writerow([item[0]]+item[1])


	with open(dirName+'/weighted_posterior_mesh_'+dt+'.csv', 'w') as file:
		writer = csv.writer(file,delimiter=' ')
		for item in list_weighted_posterior_all_time:
			print item
			writer.writerow([item[0]]+item[1])

	with open(dirName+'/robot_pose_'+dt+'.csv', 'w') as file:
		writer = csv.writer(file,delimiter=' ')
		for item in list_robot_pose:
			writer.writerow([item[0]]+item[1])

def main(args):
	global gas_meas, robot_pose, iteration_truth
	refresh_posterior_after_source_detected = False
	goal_is_the_largest_accumulated_time = True
	iteration_truth = 0
	room_source = list(np.arange(1,51,1)) #[1,2,3,4,5,6,7,8,9,10,11,12,13]
	#source_at_room = (12,45.5,3.6)
	#source_at_room = (13,48.5,3.3)
	#source_at_room = (28,22.0,11.5)
	#source_at_room = (29,24.0,12.0)
	#source_at_room = (38,1.0,14.5)
	#source_at_room = (50,26.0,21.0)
	#source_at_room = (50,26.0,20.0)
	total_simulation_time = 1200
	time_interval = 300

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
	initial_iteration = int(rospy.get_param('~initial_iteration','120'))
	source_room_number = int(rospy.get_param('~source_room_number','1'))
	source_location_x = float(rospy.get_param('~source_location_x','1.0'))
	source_location_y = float(rospy.get_param('~source_location_y','7.0'))
	source_location_z = float(rospy.get_param('~source_location_z','0.5'))
	use_time_estimation = bool(rospy.get_param('~use_time_estimation',True))
	plot_time_sampling = measurement_time_sampling*4
	min_sensor_time_sampling = 0.25
	source_at_room = (source_room_number,source_location_x,source_location_y)
	time_gas_started = initial_iteration

	if(not use_time_estimation):
		time_variant = list(np.arange(1,2,1))
	else:
		time_variant = list(np.arange(1,21,1)) #minute 1-20 every 1 minute
		#time_variant = list(np.arange(1,21,5)) #minute 1-20 every 1 minute
	
	print "simulation_folder: ",simulation_folder
	start_measurement=rospy.get_time()

	# subscriber declaration
	gas_meas_sub = rospy.Subscriber("/PID/Sensor_reading",gas_sensor,queue_size = 10,callback=gas_meas_cb)
	pose_sub = rospy.Subscriber("/odom",Odometry,queue_size = 10,callback=pose_cb)
	iteration_truth_sub = rospy.Subscriber("/iteration_truth",Int32,queue_size = 10,callback=iteration_truth_cb)

	goal_pub = rospy.Publisher("/move_base/goal",MoveBaseActionGoal,queue_size=10)
	path_pub_50 = rospy.Publisher("/path_50",Path,queue_size=10)
	path_pub_75 = rospy.Publisher("/path_75",Path,queue_size=10)
	path_pub_100 = rospy.Publisher("/path_100",Path,queue_size=10)
	path_pub_125 = rospy.Publisher("/path_125",Path,queue_size=10)
	path_pub_150 = rospy.Publisher("/path_150",Path,queue_size=10)

	
	
	
	file_name = simulation_folder+'/OccupancyGrid3Droom.csv'
	with open(file_name) as csv_file:
		csv_reader = csv.reader(csv_file, delimiter=',')
		room_matrix = list(csv_reader)

	diagram4 = GridWithWeights(len(room_matrix), len(room_matrix[0]))
	diagram4.walls = []
	for i in range(len(room_matrix)):
		for j in range(len(room_matrix[0])):
			if(int(room_matrix[i][j]) == 0):
				diagram4.walls.append((i,j))
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
		graph_file  = list(csv_reader)

	file_name = simulation_folder+'/distance_table.csv'
	with open(file_name) as csv_file:
		csv_reader = csv.reader(csv_file, delimiter=' ')
		distance_table  = list(csv_reader)

	file_name = '/home/yaqub/result IROS/gas_strength'+'/50 robot_pose_2020-03-01 16:47:11.200696.csv'
	with open(file_name) as csv_file:
		csv_reader = csv.reader(csv_file, delimiter=' ')
		list_50  = list(csv_reader)
	
	file_name = '/home/yaqub/result IROS/gas_strength'+'/75 robot_pose_2020-03-01 16:51:52.830975.csv'
	with open(file_name) as csv_file:
		csv_reader = csv.reader(csv_file, delimiter=' ')
		list_75  = list(csv_reader)

	file_name = '/home/yaqub/result IROS/gas_strength'+'/100_robot_pose_2020-03-01 15:05:05.902364.csv'
	with open(file_name) as csv_file:
		csv_reader = csv.reader(csv_file, delimiter=' ')
		list_100  = list(csv_reader)

	file_name = '/home/yaqub/result IROS/gas_strength'+'/125 robot_pose_2020-03-01 17:26:46.667345.csv'
	with open(file_name) as csv_file:
		csv_reader = csv.reader(csv_file, delimiter=' ')
		list_125  = list(csv_reader)

	file_name = '/home/yaqub/result IROS/gas_strength'+'/150 robot_pose_2020-03-01 17:29:45.215084.csv'
	with open(file_name) as csv_file:
		csv_reader = csv.reader(csv_file, delimiter=' ')
		list_150  = list(csv_reader)

	'''
	# create look up table
	list_of_distance = []
	env_min_x=-0.0004
	env_min_y=-0.1456
	environment_cell_size = 0.2
	for i in range(len(room_source)):
		list_of_distance.append([])
		for j in range(len(room_source)):
			print i,j
			start_x = float(centroid_room_source[i][0])
			start_y = float(centroid_room_source[i][1])
			grid_pos_x = int(np.ceil((start_x-env_min_x)/environment_cell_size))
			grid_pos_y = int(np.ceil((start_y-env_min_y)/environment_cell_size))
			robot_at_room = int(room_matrix[grid_pos_x][grid_pos_y])
			if(robot_at_room == 0):
				grid_pos_x = int(np.ceil((start_x-env_min_x)/environment_cell_size))
				grid_pos_y = int(np.floor((start_y-env_min_y)/environment_cell_size))
				robot_at_room = int(room_matrix[grid_pos_x][grid_pos_y])
				if(robot_at_room == 0):
					grid_pos_x = int(np.floor((start_x-env_min_x)/environment_cell_size))
					grid_pos_y = int(np.ceil((start_y-env_min_y)/environment_cell_size))
					robot_at_room = int(room_matrix[grid_pos_x][grid_pos_y])
					if(robot_at_room == 0):
						grid_pos_x = int(np.floor((start_x-env_min_x)/environment_cell_size))
						grid_pos_y = int(np.floor((start_y-env_min_y)/environment_cell_size))
						#robot_at_room = int(room_matrix[grid_pos_x][grid_pos_y])
			start = (grid_pos_x,grid_pos_y)
			goal_x = float(centroid_room_source[j][0])
			goal_y = float(centroid_room_source[j][1])
			grid_pos_x = int(np.ceil((goal_x-env_min_x)/environment_cell_size))
			grid_pos_y = int(np.ceil((goal_y-env_min_y)/environment_cell_size))
			robot_at_room = int(room_matrix[grid_pos_x][grid_pos_y])
			if(robot_at_room == 0):
				grid_pos_x = int(np.ceil((goal_x-env_min_x)/environment_cell_size))
				grid_pos_y = int(np.floor((goal_y-env_min_y)/environment_cell_size))
				robot_at_room = int(room_matrix[grid_pos_x][grid_pos_y])
				if(robot_at_room == 0):
					grid_pos_x = int(np.floor((goal_x-env_min_x)/environment_cell_size))
					grid_pos_y = int(np.ceil((goal_y-env_min_y)/environment_cell_size))
					robot_at_room = int(room_matrix[grid_pos_x][grid_pos_y])
					if(robot_at_room == 0):
						grid_pos_x = int(np.floor((goal_x-env_min_x)/environment_cell_size))
						grid_pos_y = int(np.floor((goal_y-env_min_y)/environment_cell_size))
			goal = (grid_pos_x,grid_pos_y)
			came_from, cost_so_far = a_star_search(diagram4, start, goal)
			travel_cost = len(reconstruct_path(came_from, start=start, goal=goal))*0.2
			list_of_distance[-1].append(travel_cost)
	print list_of_distance
	print len(list_of_distance)
	print len(list_of_distance[0])
	with open(simulation_folder+'/distance_table.csv', 'w') as file:
		writer = csv.writer(file,delimiter=' ')
		for item in list_of_distance:
			writer.writerow(item)
	exit()
	'''
	list_graph=[]
	vert = 1
	for i in graph_file:
	    for j in range(len(i)):
	        list_graph.append((str(vert),i[j],1))
	    vert=vert+1
	graph = Graph(list_graph)

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
	
	room_weighted_marker_array_topic = 'room_weighted_marker'
	room_weighted_marker_array_pub = rospy.Publisher(room_weighted_marker_array_topic, MarkerArray,queue_size=10)
	room_weighted_marker_array = MarkerArray()
	room_weighted_marker_array.markers = []
	for i in room_source:
		room_weighted_marker = Marker()
		room_weighted_marker.header.frame_id = "/map"
		room_weighted_marker.header.stamp=rospy.Time.now()
		room_weighted_marker.id = i
		room_weighted_marker.type = room_weighted_marker.CUBE
		room_weighted_marker.action = room_weighted_marker.ADD
		room_weighted_marker.scale.x = list_size_room[i-1][0]-list_size_room[i-1][1]
		room_weighted_marker.scale.y = list_size_room[i-1][2]-list_size_room[i-1][3]
		room_weighted_marker.scale.z = 3
		room_weighted_marker.color.r =255.0/255.0
		room_weighted_marker.color.g= 255.0/255.0
		room_weighted_marker.color.b =55.0/255.0
		room_weighted_marker.color.a =0.85
		#room_marker.lifetime = rospy.Duration(0)
		room_weighted_marker.pose.position.x = (list_size_room[i-1][0]+list_size_room[i-1][1])/2.0
		room_weighted_marker.pose.position.y = (list_size_room[i-1][2]+list_size_room[i-1][3])/2.0
		room_weighted_marker.pose.position.z = 1.5
		room_weighted_marker.pose.orientation.x = 0.0
		room_weighted_marker.pose.orientation.y = 0.0
		room_weighted_marker.pose.orientation.z = 0.0
		room_weighted_marker.pose.orientation.w = 1.0
		room_weighted_marker_array.markers.append(room_weighted_marker)

	if(use_time_estimation):
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
	
	path_display_50 = Path()
	path_display_50.header.frame_id = "/map"
	path_display_50.header.stamp = rospy.Time.now()
	path_display_50.poses = [] 
	for item in list_50:
		pose_temp = PoseStamped()
		pose_temp.pose.position.x = float(item[1])
		pose_temp.pose.position.y = float(item[2])
		pose_temp.pose.position.z = 0.2
		path_display_50.poses.append(pose_temp)


	path_display_75 = Path()
	path_display_75.header.frame_id = "/map"
	path_display_75.header.stamp = rospy.Time.now()
	path_display_75.poses = [] 
	for item in list_75:
		pose_temp = PoseStamped()
		pose_temp.pose.position.x = float(item[1])
		pose_temp.pose.position.y = float(item[2])
		pose_temp.pose.position.z = 0.2
		path_display_75.poses.append(pose_temp)


	path_display_100 = Path()
	path_display_100.header.frame_id = "/map"
	path_display_100.header.stamp = rospy.Time.now()
	path_display_100.poses = [] 
	for item in list_100:
		pose_temp = PoseStamped()
		pose_temp.pose.position.x = float(item[1])
		pose_temp.pose.position.y = float(item[2])
		pose_temp.pose.position.z = 0.2
		path_display_100.poses.append(pose_temp)


	path_display_125 = Path()
	path_display_125.header.frame_id = "/map"
	path_display_125.header.stamp = rospy.Time.now()
	path_display_125.poses = [] 
	for item in list_125:
		pose_temp = PoseStamped()
		pose_temp.pose.position.x = float(item[1])
		pose_temp.pose.position.y = float(item[2])
		pose_temp.pose.position.z = 0.2
		path_display_125.poses.append(pose_temp)


	path_display_150 = Path()
	path_display_150.header.frame_id = "/map"
	path_display_150.header.stamp = rospy.Time.now()
	path_display_150.poses = [] 
	for item in list_150:
		pose_temp = PoseStamped()
		pose_temp.pose.position.x = float(item[1])
		pose_temp.pose.position.y = float(item[2])
		pose_temp.pose.position.z = 0.2
		path_display_150.poses.append(pose_temp)

	start_publish = rospy.get_time()
	while not rospy.is_shutdown():
		if(rospy.get_time()-start_publish > 1.0):
			path_pub_50.publish(path_display_50)
			path_pub_75.publish(path_display_75)
			path_pub_100.publish(path_display_100)
			path_pub_125.publish(path_display_125)
			path_pub_150.publish(path_display_150)

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

	if(not use_time_estimation):
			#dict_of_kde_pdf=read_dataset_kde_pose_only(simulation_folder,room_source)
			with open(simulation_folder+'/dict_of_kde_pose_only_pdf.obj', 'rb') as config_dictionary_file:
				dict_of_kde_pdf = pickle.load(config_dictionary_file)
			
	else:
		if(likelihood_function_used == KDE):
			#dict_of_kde_pdf=read_dataset_kde(simulation_folder,room_source,time_variant)
			#dict_of_kde_pdf=read_dataset_kde_specific_time(simulation_folder,room_source,5.0)
			with open(simulation_folder+'/dict_of_kde_pdf.obj', 'rb') as config_dictionary_file:
				dict_of_kde_pdf = pickle.load(config_dictionary_file)

		if(likelihood_function_used == Gaussian):
			dict_of_gaussian_pdf=read_dataset_gaussian(simulation_folder,room_source,time_variant)
	
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
	trajectory_length = 0
	prev_robot_pose = Pose()
	prev_robot_pose.position.x = robot_pose.position.x
	prev_robot_pose.position.y = robot_pose.position.y
	write_result = False
	list_all_time_likelihood = []
	list_all_time_posterior = []
	list_all_time_weighted_posterior = []
	list_robot_pose = []
	force_to_zero_room = []
	visited_room = set()
	while not rospy.is_shutdown():
		# get measurement as many as the sensor can and average it
		#concentration_data.append(gas_meas)
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
			if (iteration_truth-time_gas_started > time_interval*time_variant_flag[0]) and (use_time_estimation):
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
			#concentration = gas_meas
			#concentration = np.mean(concentration_data)
			concentration = (gas_meas+gas_meas*0.2*(random.random()-0.5))
			#concentration = (gas_meas+gas_meas*0.2*(random.random()-0.5))/float(1000.0)
			concentration_data = []
			grid_pos_x = int(np.ceil((robot_pose.position.x-env_min_x)/environment_cell_size))
			grid_pos_y = int(np.ceil((robot_pose.position.y-env_min_y)/environment_cell_size))
			robot_at_room = int(room_matrix[grid_pos_x][grid_pos_y])
			list_robot_pose.append((iteration_truth,[robot_pose.position.x,robot_pose.position.y]))
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
			robot_detect_gas = True
			printv(verbose,'conc: '+str(gas_meas))
			#in which room the measurement is done?
			#print "pose:",robot_pose.position.x,robot_pose.position.y
			
			printv(verbose,'room:'+str(robot_at_room)+' '+str(grid_pos_x)+' '+str(grid_pos_y))
			str_of_room = '{:02d}'.format(robot_at_room)
			#get the mean and std in room at time 1-15 min
			pos_idx = 0

			list_likelihood = []
			list_posterior = []
			if (np.sum(posterior) != 1.0):
				print "WARNING POSTERIOR SUM NOT ONE", np.sum(posterior)
				post = normalize(posterior)
				posterior = post[:]
				if(np.sum(posterior) != 1.0):
					print "WARNING POSTERIOR SUM STILL NOT ONE", np.sum(posterior)
			start_calc = rospy.get_time()
			#posterior_delta = 5e-8
			posterior_delta = 0
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
				if(not use_time_estimation):
					likelihood = calculate_likelihood_kde_pose_only(concentration,robot_at_room,rs,dict_of_kde_pdf)
					list_likelihood.append(likelihood)

					posterior[pos_idx] = likelihood*posterior[pos_idx] #/float(travel_cost)
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
							posterior[pos_idx] = likelihood*posterior[pos_idx]+posterior_delta #/float(travel_cost)
							list_likelihood.append(likelihood)
						pos_idx = pos_idx+1	
			print "time spent:",rospy.get_time()-start_calc,"sec"
			#print "likelihood",list_likelihood
			tempos=normalize(posterior)
			posterior = tempos[:]
			pos_idx=0


			list_all_time_likelihood.append((iteration_truth,list_likelihood[:]))
			list_all_time_posterior.append((iteration_truth,posterior[:]))
			#print list_all_time_posterior

			#print np.sum(list_all_time_posterior[-1][1])
			#print "sum of posterior",np.sum(posterior)
			#print "mean of posterior",np.mean(posterior)
			marginal_posterior_room = []
			marginal_posterior_room_weighted = []
			for i in range(len(room_source)):
				travel_cost = float(distance_table[i][robot_at_room-1])
				#print i,travel_cost
				#travel_cost = len(graph.dijkstra(str(robot_at_room), str(room_source[i])))
				if(i+1 is robot_at_room):
					travel_cost = 100000

				temp_list = []
				for j in range(len(time_variant)):
					temp_list.append(posterior[i*len(time_variant)+j])

				room_marker_array.markers[i].header.stamp=rospy.Time.now()
				room_marker_array.markers[i].scale.z = 3*np.sum(temp_list)
				room_marker_array.markers[i].pose.position.z = room_marker_array.markers[i].scale.z/2.0
				marginal_posterior_room.append(np.sum(temp_list))
				if i+1 in visited_room:
					#print i+1,"is in visited room",visited_room
					marginal_posterior_room_weighted.append(0)
				else:
					marginal_posterior_room_weighted.append(np.sum(temp_list)+float(alpha)/float(travel_cost))
				
				list_posterior_room[i].append(np.sum(temp_list))
				room_weighted_marker_array.markers[i].header.stamp=rospy.Time.now()
				room_weighted_marker_array.markers[i].scale.z = 3*(marginal_posterior_room_weighted[-1])
				room_weighted_marker_array.markers[i].pose.position.z = room_weighted_marker_array.markers[i].scale.z/2.0
			
			
			room_marker_array_pub.publish(room_marker_array)
			list_all_time_weighted_posterior.append((iteration_truth,marginal_posterior_room_weighted))
			
			room_weighted_marker_array_pub.publish(room_weighted_marker_array)

			max_post_room_accumulated_idx = np.argmax(marginal_posterior_room_weighted)
			print "Largest weighted posterior:",max_post_room_accumulated_idx,marginal_posterior_room_weighted[max_post_room_accumulated_idx]
			#print 'robot room:',robot_at_room
			#print 'robot pose',(robot_pose.position.x,robot_pose.position.y)
			#print 'robot at room:',float(door_room_source [robot_at_room-1][0]),float(door_room_source [robot_at_room-1][1])
			#print "dist to center:", euclidean_distance((robot_pose.position.x,robot_pose.position.y),(float(door_room_source [robot_at_room-1][0]),float(door_room_source [robot_at_room-1][1])))
			#print 'goal', float(door_room_source [max_room_source-1][0]),float(door_room_source [max_room_source-1][1])
			#print 'goal', float(door_room_source [max_post_room_accumulated_idx][0]),float(door_room_source [max_post_room_accumulated_idx][1])
			if((robot_at_room in room_source) and (robot_at_room != source_at_room[0])): # and (euclidean_distance((robot_pose.position.x,robot_pose.position.y),(float(door_room_source [robot_at_room-1][0]),float(door_room_source [robot_at_room-1][1]))) < goal_treshold):
				print "Force posterior at room-",robot_at_room,"to be zero"
				#force_to_zero_room.append(robot_at_room)
				visited_room.add(int(robot_at_room))
				if(visual_check):
					for j in range(len(time_variant)):
						#print "force debug:",robot_at_room,(robot_at_room-1)*len(time_variant)+j
						posterior[(robot_at_room-1)*len(time_variant)+j] = 0.0
					#pass
			#print "p(room) = zero",force_to_zero_room
			
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
			goal = MoveBaseActionGoal()
			#goal.header.seq = 
			goal.goal.target_pose.header.stamp = rospy.Time.now()
			goal.goal.target_pose.header.frame_id = "/map" 
			#goal.goal.target_pose.pose.position.x = float(door_room_source [max_room_source-1][0])
			#goal.goal.target_pose.pose.position.y = float(door_room_source [max_room_source-1][1])
			
			if(goal_is_the_largest_accumulated_time):
				max_idx_for_goal = max_post_room_accumulated_idx
			else:
				max_idx_for_goal = max_post_room_source-1

			if(robot_at_room == source_at_room[0]):
				print "Robot arrive at the source room:",robot_at_room
				#mission_completed = True
				for i in room_source:
					if(i != robot_at_room):
						for j in range(len(time_variant)):
							#print 'check:',(i-1)*len(time_variant)+j
							#posterior[(i-1)*len(time_variant)+j] = 0.0
							pass
				goal.goal.target_pose.pose.position.x = float(centroid_room_source [max_idx_for_goal][0]) 
				goal.goal.target_pose.pose.position.y = float(centroid_room_source [max_idx_for_goal][1])
				measurement_time_sampling = 1.0
			else:
				print "goal in room",max_idx_for_goal+1
				goal.goal.target_pose.pose.position.x = float(door_room_source [max_idx_for_goal][0])
				goal.goal.target_pose.pose.position.y = float(door_room_source [max_idx_for_goal][1])
			goal.goal.target_pose.pose.position.z = 0.0
			goal.goal.target_pose.pose.orientation.x = 0.0
			goal.goal.target_pose.pose.orientation.y = 0.0
			goal.goal.target_pose.pose.orientation.z = 0.0
			goal.goal.target_pose.pose.orientation.w = 1.0
			goal_marker.header.stamp=rospy.Time.now()
			goal_marker.pose.position.x = goal.goal.target_pose.pose.position.x
			goal_marker.pose.position.y = goal.goal.target_pose.pose.position.y
			goal_marker.pose.position.z = 0.1 
			goal_marker_pub.publish(goal_marker)
			if(euclidean_distance((robot_pose.position.x,robot_pose.position.y),(float(centroid_room_source [source_at_room[0]-1][0]),float(centroid_room_source [source_at_room[0]-1][1]))) < goal_treshold):
				print "Robot arrive at the source point"
				if(not write_result):
					write_result = True
					write_result_to_csv_file(list_robot_pose,list_all_time_likelihood,list_all_time_posterior,list_all_time_weighted_posterior,simulation_folder+gas_source_folder)
				mission_completed = True
			
			if(not mission_completed):
				goal_pub.publish(goal)
			room_marker.header.stamp=rospy.Time.now()

			if(use_time_estimation):
				for j in range(len(time_variant)):
					time_marker_array.markers[j].scale.z = 3*posterior[(source_at_room[0]-1)*len(time_variant)+j]
					time_marker_array.markers[j].pose.position.z = 3+(time_marker_array.markers[j].scale.z)/2.0
				time_marker_array_pub.publish(time_marker_array)

			for j in range(len(room_source)*len(time_variant)):
				list_posterior_room_time[j].append(posterior[j])
			list_time_secs.append(time_now_secs)
			idx=idx+1
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
