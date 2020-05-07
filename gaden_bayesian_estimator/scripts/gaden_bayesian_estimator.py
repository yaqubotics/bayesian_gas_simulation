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
from datetime import datetime
#import keyboard

verbose = True

def printv(ver,str_):
	if(ver):
		print str_ 

def calculate_likelihood(x,mean,std):
	if(std == 0):
		std = 0.000001
	return (1.0/(std*np.sqrt(2*np.pi)))*np.exp(-0.5*(((x-mean)/std)**2))#+(random.random())*0.001

def normalize(a):
	s = []
	for i in a:
		norm = float(i)/sum(a)+0.000001
		#if norm == 0:
		#	norm = 0.0000001
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
	iteration_truth = 0
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
	print "simulation_folder: ",simulation_folder
	start_measurement=rospy.get_time()

	# subscriber declaration
	gas_meas_sub = rospy.Subscriber("/PID/Sensor_reading",gas_sensor,queue_size = 10,callback=gas_meas_cb)
	pose_sub = rospy.Subscriber("/odom",Odometry,queue_size = 10,callback=pose_cb)
	iteration_truth_sub = rospy.Subscriber("/iteration_truth",Int32,queue_size = 10,callback=iteration_truth_cb)

	goal_pub = rospy.Publisher("/move_base/goal",MoveBaseActionGoal,queue_size=10)

	room_source = [1,2,3,4,5,6,7,8,9,10,11,12,13]
	time_variant = list(np.arange(1,21,1)) #minute 1-20 every 1 minute
	source_at_room = (12,45.5,3.6)

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
	
	time_marker_array_topic = 'time_marker'
	time_marker_array_pub = rospy.Publisher(time_marker_array_topic, MarkerArray,queue_size=10)
	time_marker_array = MarkerArray()
	time_marker_array.markers = []
	for i in room_source:
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
			y1 = list_size_room[i-1][2]+(list_size_room[i-1][3]-list_size_room[i-1][2])*(j-1)/len(time_variant)
			y2 = list_size_room[i-1][2]+(list_size_room[i-1][3]-list_size_room[i-1][2])*(j)/len(time_variant)
			
			time_marker.pose.position.x = (list_size_room[i-1][0]+list_size_room[i-1][1])/2.0
			time_marker.pose.position.y = (y1+y2)/2.0
			time_marker.pose.position.z = 1.5
			time_marker.pose.orientation.x = 0.0
			time_marker.pose.orientation.y = 0.0
			time_marker.pose.orientation.z = 0.0
			time_marker.pose.orientation.w = 1.0
			time_marker_array.markers.append(time_marker)
	
	
	#print room_marker_array
	robot_detect_gas = False
	max_room_source = 22
	max_post_room_accumulated_idx = max_room_source-1
	prev_goal = 0
	goal_treshold = 1.0
	concentration_data = []
	start_plot=rospy.get_time()
	while(iteration_truth == 0):
		pass
	list_time_secs = [iteration_truth]
	while not rospy.is_shutdown():
		# get measurement as many as the sensor can and average it
		concentration_data.append(gas_meas)
		if(rospy.get_time()-start_measurement > measurement_time_sampling):
			start_measurement=rospy.get_time()
			time_now_secs = iteration_truth
			print "Iteration truth: ",iteration_truth
			#concentration = gas_meas
			concentration = np.mean(concentration_data)
			concentration_data = []
			grid_pos_x = int(np.ceil((robot_pose.position.x-env_min_x)/environment_cell_size))
			grid_pos_y = int(np.ceil((robot_pose.position.y-env_min_y)/environment_cell_size))
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
			for rs in room_source:
				str_of_rs = '{:02d}'.format(rs)
				#find file
				all_file = []
				the_list_dir_path = simulation_folder+'/analysis/gas_source_'+str_of_rs+'/meas_at_'+str_of_room+'/' 
				filenames= os.listdir (the_list_dir_path)
				
				for file in filenames:
					if("mean_std_min" in file):
						mean_std_csv_file = file
				file_name = the_list_dir_path+mean_std_csv_file
				list_mean_time = []
				list_std_time = []
				list_post_time = []
				with open(file_name) as csv_file:
					csv_reader = csv.reader(csv_file, delimiter=' ')
					csv_reader = list(csv_reader)
					for tv in time_variant:
						mean = float(csv_reader[tv][1])
						std = float(csv_reader[tv][2])	
						likelihood=calculate_likelihood(concentration,mean,std)
						posterior[pos_idx] = likelihood*posterior[pos_idx]
						if(likelihood > 1):
							print 'warning likelihood',likelihood,mean,std,concentration
							print 'room:',pos_idx/len(time_variant)
							print 'time:',pos_idx%len(time_variant)
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
			print 'robot room:',robot_at_room
			print 'robot pose',(robot_pose.position.x,robot_pose.position.y)
			print 'robot at room:',float(door_room_source [robot_at_room-1][0]),float(door_room_source [robot_at_room-1][1])
			print "dist to center:", euclidean_distance((robot_pose.position.x,robot_pose.position.y),(float(door_room_source [robot_at_room-1][0]),float(door_room_source [robot_at_room-1][1])))
			#print 'goal', float(door_room_source [max_room_source-1][0]),float(door_room_source [max_room_source-1][1])
			print 'goal', float(door_room_source [max_post_room_accumulated_idx][0]),float(door_room_source [max_post_room_accumulated_idx][1])
			if(robot_at_room in room_source) and (max_room_source != source_at_room[0]) and (euclidean_distance((robot_pose.position.x,robot_pose.position.y),(float(door_room_source [robot_at_room-1][0]),float(door_room_source [robot_at_room-1][1]))) < goal_treshold):
				"Force posterior at room-",robot_at_room,"to be zero"
				for j in range(len(time_variant)):
					posterior[(robot_at_room-1)*len(time_variant)+j] = 0.0
			max_post_idx = np.argmax(posterior)
			max_room_source = room_source[max_post_idx / len(time_variant)]
			max_time_source = max_post_idx % len(time_variant) + 1
			list_max_posterior_room.append(room_source[max_post_idx / len(time_variant)])
			print "largest posterior in room-",max_room_source,"at minute-",max_time_source

			# publish to that room
			goal = MoveBaseActionGoal()
			#goal.header.seq = 
			goal.goal.target_pose.header.stamp = rospy.Time.now()
			goal.goal.target_pose.header.frame_id = "/map" 
			#goal.goal.target_pose.pose.position.x = float(door_room_source [max_room_source-1][0])
			#goal.goal.target_pose.pose.position.y = float(door_room_source [max_room_source-1][1])
			if(robot_at_room == source_at_room[0]):
				goal.goal.target_pose.pose.position.x = float(outlet_room_source [max_post_room_accumulated_idx][0]) 
				goal.goal.target_pose.pose.position.y = float(outlet_room_source [max_post_room_accumulated_idx][1])
				measurement_time_sampling = 0.5
			else:
				goal.goal.target_pose.pose.position.x = float(door_room_source [max_post_room_accumulated_idx][0])
				goal.goal.target_pose.pose.position.y = float(door_room_source [max_post_room_accumulated_idx][1])
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
			goal_pub.publish(goal)
			room_marker.header.stamp=rospy.Time.now()

			for i in range(len(room_source)):
				for j in range(len(time_variant)):
					time_marker_array.markers[i*len(time_variant)+j].scale.z = 3*posterior[i*len(time_variant)+j]
					time_marker_array.markers[i*len(time_variant)+j].pose.position.z = 3+(time_marker_array.markers[i*len(time_variant)+j].scale.z)/2.0

			time_marker_array_pub.publish(time_marker_array)

			for j in range(len(room_source)*len(time_variant)):
				list_posterior_room_time[j].append(posterior[j])
			list_time_secs.append(time_now_secs)
			idx=idx+1
		if(rospy.get_time()-start_plot > plot_time_sampling):
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
				ax.plot(list_time_secs,list_posterior_room_time[(room_source[0]-1)*len(time_variant)+i])
			plot_title = 'Probability source in room and time '+str(source_at_room[0])+' ('+str(source_at_room[1])+','+str(source_at_room[2])+') tf:'+str(iteration_truth)
			plt.title(plot_title)
			plt.xlabel('sec')
			plt.ylabel('P')
			legend_p = list(time_variant)
			for lp in range(len(legend_p)):
				legend_p[lp]='Minute '+str(legend_p[lp])

			plt.legend(legend_p)
			plt.savefig(simulation_folder+'/plot/'+plot_title+' '+str(date_time_now)+'.png')
			plt.close()
			
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

def printv(ver,str_):
	if(ver):
		print str_ 

def calculate_likelihood(x,mean,std):
	if(std == 0):
		std = 0.000001
	return (1.0/(std*np.sqrt(2*np.pi)))*np.exp(-0.5*(((x-mean)/std)**2))#+(random.random())*0.001

def calculate_likelihood_dis(x,mean,meanx):
	return np.abs(mean-x)

def calculate_likelihood2(x,mean,std):
	#x = x+(random.random())*0.01
	return (1.0/(np.log(std)*np.sqrt(2*np.pi)))*np.exp(-0.5*(((np.log(x)-np.log(mean))/np.log(std))**2))#+(random.random())*0.01

def belief(p,prev_lod):
	lod = np.log(p/(1.0-p))+0.5+prev_lod
	return (1-1.0/np.exp(lod),lod)

def normalize(a):
	s = []
	for i in a:
		norm = float(i)/sum(a)+0.000001
		#if norm == 0:
		#	norm = 0.0000001
		s.append(norm)
	return s
current_folder = os.getcwd()
#test_folder = '/csv_file/pdf_Test_ts_1_nfs_100_ppm_5000_fis_10_fgg_0.16_fnss_0.1_fnsb_0.3_st_1202_sourcePose_29.00_2.00_0.50'
#test_folder = '/csv_file/pdf_Test_ts_1_nfs_100_ppm_5000_fis_10_fgg_0.16_fnss_0.1_fnsb_0.3_st_1202_sourcePose_26.00_2.50_2.00'
#test_folder = '/csv_file/pdf_Test_ts_1_nfs_100_ppm_2000_fis_10_fgg_0.16_fnss_0.1_fnsb_0.3_st_1202_sourcePose_29.00_2.00_0.50'
#test_folder = '/csv_file/pdf_r_01_C_ts_1_nfs_100_ppm_5000_fis_10_fgg_0.16_fnss_0.1_fnsb_0.3_st_1202_sourcePose_2.70_3.30_0.50'
#test_folder = '/csv_file/pdf_r_02_C_ts_1_nfs_100_ppm_5000_fis_10_fgg_0.16_fnss_0.1_fnsb_0.3_st_1202_sourcePose_5.50_3.30_0.50'
#test_folder = '/csv_file/pdf_r_03_C_ts_1_nfs_100_ppm_5000_fis_10_fgg_0.16_fnss_0.1_fnsb_0.3_st_1202_sourcePose_8.90_3.30_0.50'
#test_folder = '/csv_file/pdf_r_04_C_ts_1_nfs_100_ppm_5000_fis_10_fgg_0.16_fnss_0.1_fnsb_0.3_st_1202_sourcePose_11.90_3.30_0.50'
#test_folder = '/csv_file/pdf_r_05_C_ts_1_nfs_100_ppm_5000_fis_10_fgg_0.16_fnss_0.1_fnsb_0.3_st_1202_sourcePose_16.70_3.60_0.50'
test_folder = '/csv_file/pdf_r_06_C_ts_1_nfs_100_ppm_5000_fis_10_fgg_0.16_fnss_0.1_fnsb_0.3_st_1202_sourcePose_21.30_3.10_0.50'
#test_folder = '/csv_file/pdf_r_07_UR_ts_1_nfs_100_ppm_5000_fis_10_fgg_0.16_fnss_0.1_fnsb_0.3_st_1202_sourcePose_26.00_5.00_0.50'
#test_folder = '/csv_file/pdf_r_07_UL_ts_1_nfs_100_ppm_5000_fis_10_fgg_0.16_fnss_0.1_fnsb_0.3_st_1202_sourcePose_23.00_4.50_0.50'
#test_folder = '/csv_file/pdf_r_07_C_ts_1_nfs_100_ppm_5000_fis_10_fgg_0.16_fnss_0.1_fnsb_0.3_st_1202_sourcePose_24.50_2.50_0.50'
#test_folder = '/csv_file/pdf_r_07_BR_ts_1_nfs_100_ppm_5000_fis_10_fgg_0.16_fnss_0.1_fnsb_0.3_st_1202_sourcePose_26.00_1.00_0.50'
#test_folder = '/csv_file/pdf_r_07_BL_ts_1_nfs_100_ppm_5000_fis_10_fgg_0.16_fnss_0.1_fnsb_0.3_st_1202_sourcePose_23.00_1.00_0.50'
#test_folder = '/csv_file/pdf_r_08_BR_ts_1_nfs_100_ppm_5000_fis_10_fgg_0.16_fnss_0.1_fnsb_0.3_st_1202_sourcePose_32.00_1.00_0.50'
#test_folder = '/csv_file/pdf_r_08_BL_ts_1_nfs_100_ppm_5000_fis_10_fgg_0.16_fnss_0.1_fnsb_0.3_st_1202_sourcePose_28.50_1.00_0.50'
#test_folder = '/csv_file/pdf_r_08_C_ts_1_nfs_100_ppm_5000_fis_10_fgg_0.16_fnss_0.1_fnsb_0.3_st_1202_sourcePose_30.60_3.30_0.50'
#test_folder = '/csv_file/pdf_r_08_UL_ts_1_nfs_100_ppm_5000_fis_10_fgg_0.16_fnss_0.1_fnsb_0.3_st_1202_sourcePose_28.50_5.00_0.50'
#test_folder = '/csv_file/pdf_r_08_UR_ts_1_nfs_100_ppm_5000_fis_10_fgg_0.16_fnss_0.1_fnsb_0.3_st_1202_sourcePose_32.00_5.00_0.50'
#test_folder = '/csv_file/pdf_r_09_C_ts_1_nfs_100_ppm_5000_fis_10_fgg_0.16_fnss_0.1_fnsb_0.3_st_1202_sourcePose_35.10_3.60_0.50'
#test_folder = '/csv_file/pdf_r_10_C_ts_1_nfs_100_ppm_5000_fis_10_fgg_0.16_fnss_0.1_fnsb_0.3_st_1202_sourcePose_38.50_3.60_0.50'
#test_folder = '/csv_file/pdf_r_11_C_ts_1_nfs_100_ppm_5000_fis_10_fgg_0.16_fnss_0.1_fnsb_0.3_st_1202_sourcePose_42.00_3.60_0.50'
#test_folder = '/csv_file/pdf_r_12_C_ts_1_nfs_100_ppm_5000_fis_10_fgg_0.16_fnss_0.1_fnsb_0.3_st_1202_sourcePose_45.50_3.60_0.50'
#test_folder = '/csv_file/pdf_r_13_P_ts_1_nfs_100_ppm_5000_fis_10_fgg_0.16_fnss_0.1_fnsb_0.3_st_1202_sourcePose_48.50_1.50_0.50'
#test_folder = '/csv_file/pdf_r_13_Q_ts_1_nfs_100_ppm_5000_fis_10_fgg_0.16_fnss_0.1_fnsb_0.3_st_1202_sourcePose_48.50_3.30_0.50'
#test_folder = '/csv_file/pdf_r_13_R_ts_1_nfs_100_ppm_5000_fis_10_fgg_0.16_fnss_0.1_fnsb_0.3_st_1202_sourcePose_48.50_5.00_0.50'

#room_source = [1,2,3,4,5,6,7,8,9,10,11,12,13]
room_source = [1,2,3,4,5,6,7,8,9,10,11,12,13]
source_at_room = (6,21.3,3.1)
time_variant = list(np.arange(1,16,1)) #minute 1-15 every 1 minute
#robot_path_time = np.arange(15,20,1) 
posterior = []
list_max_posterior_room = []
list_max_posterior_room2 = []
list_max_posterior_room3 = []
list_max_posterior_room4 = []
list_max_posterior_room5 = []
list_max_posterior_room6 = []
list_max_posterior_room7 = []
list_max_posterior_room8 = []
list_max_posterior_room9 = []
list_max_posterior_room10 = []
for i in range(len(room_source)*len(time_variant)):
	posterior.append(1.0/(len(room_source)*len(time_variant)))

#ROOM_18="room_number:=18 P_x:=20 P_y:=7.2 Q_x:=22 Q_y:=7.2 R_x:=24 R_y:=7.2"
#ROOM_19="room_number:=19 P_x:=26 P_y:=7.2 Q_x:=28 Q_y:=7.2 R_x:=30 R_y:=7.2"

noise_measurement=0.0
cell_size = 0.2
meas_time_min = 19

meas_time_secs = meas_time_min*60
meas_time_min_start = 15
meas_time_secs_start = meas_time_min_start*60
secs_per_point = 0.2
# room 17-18-19-20
#meas_pos = [(14,7.2),(16,7.2),(18,7.2),(20,7.2),(22,7.2),(24,7.2),(26,7.2),(28,7.2),(30,7.2),(32,7.2),(34,7.2),(36,7.2)]
'''(22,7),(23,7.4),(24,7),(25,7.4),'''
#zigzag
import random
#meas_pos = [(26,7.0),(27,7.4),(28,7.0),(29,7.4),(30,7.0),(31,7.4),(32,7.0),(33,7.4)]

'''
meas_pos_x = np.arange(1,49,0.2)
meas_pos = []
for i in meas_pos_x:
	meas_pos.append((i,random.randint(35,37)*0.2))
'''
meas_pos_x_temp = list(np.arange(1,52,0.2))

meas_pos_x = meas_pos_x_temp+meas_pos_x_temp[::-1]#+list(np.arange(1,15,0.1))
meas_pos = []
for i in meas_pos_x[:len(meas_pos_x)/2+1]:
	meas_pos.append((i,35*0.2))
for i in meas_pos_x[len(meas_pos_x)/2+1:len(meas_pos_x)+1]:
	meas_pos.append((i,38*0.2))

#meas_pos.reverse()
#meas_pos = [(14,7.2),(16,7.2),(18,7.2),(20,7.2),(22,7.2),(24,7.2),(26,7.2),(28,7.2),(30,7.2),(32,7.2),(34,7.2),(36,7.2),(34,7.2),(32,7.2),(30,7.2),(28,7.2),(26,7.2),(24,7.2),(22,7.2),(20,7.2),(18,7.2),(16,7.2),(14,7.2)]
#meas_pos = [(14,7.2),(16,7.2),(18,7.2),(20,7.2),(22,7.2),(24,7.2),(26,7.2),(28,7.2),(30,7.2),(32,7.2),(32,6),(32,4),(31,4),(31,6),(31,6.5),(30,6.5),(28,6.5),(27,6.5),(26,6.5),(25.5,5)]
#ROOM_07="room_number:=07 C_x:=24.5 C_y:=2.5 UL_x:=23 UL_y:=4.5 UR_x:=26 UR_y:=5 BL_x:=23 BL_y:=1 BR_x:=26 BR_y:=1"

#meas_pos = [(25.5,2.5),(23.5,4),(26,4.2)]
#meas_pos.reverse()
meas_time_secs = meas_time_secs_start
meas_time_secs_float = meas_time_secs_start
list_posterior = []
#print posterior
list_likelihood = []
for i in range(len(room_source)*len(time_variant)):
	list_posterior.append([posterior[i]])
	list_likelihood.append(0)
list_time = [meas_time_secs]
prev_lod = 0.5
err_time_pred = []
list_max_likelihood = []
for i in meas_pos:
	#get the concentration at time 11 from GADEN_Test csv
	grid_pos_x = int(np.floor(i[0]/cell_size))
	grid_pos_y = int(np.floor(i[1]/cell_size))
	#open the csv file
	meas_time_secs = int(np.floor(meas_time_secs_float))
	#print meas_time_secs
	str_of_time_secs = '{:4d}'.format(meas_time_secs)

	file_name = current_folder+test_folder+'/PDF_gasType_10_iter_'+str_of_time_secs+'.csv'
	a_list = []
	#print file_name
	with open(file_name) as csv_file:
		csv_reader = csv.reader(csv_file, delimiter=' ')
		csv_reader = list(csv_reader)
		concentration = float(csv_reader[grid_pos_x][grid_pos_y])#+(random.random()-0.5)*noise_measurement
	if(concentration > 0):
		printv(verbose,'time: '+str(meas_time_secs))
		printv(verbose,'conc: '+str(concentration))
		#in which room the measurement is done?
		file_name = current_folder+'/OccupancyGrid3Droom.csv'
		with open(file_name) as csv_file:
			csv_reader = csv.reader(csv_file, delimiter=',')
			csv_reader = list(csv_reader)
			room = int(csv_reader[grid_pos_x][grid_pos_y])
		printv(verbose,'room:'+str(room)+' '+str(grid_pos_x)+' '+str(grid_pos_y))
		str_of_room = '{:02d}'.format(room)
		#get the mean and std in room at time 1-15 min
		mean = []
		std = []
		pos_idx = 0

		list_likelihood = []
		for rs in room_source:
			# open csv file
			str_of_rs = '{:02d}'.format(rs)
			#find file
			all_file = []
			the_list_dir_path = current_folder+'/analysis/gas_source_'+str_of_rs+'/meas_at_'+str_of_room+'/' 
			filenames= os.listdir (the_list_dir_path)
			
			for file in filenames:
				if("mean_std_min" in file):
					mean_std_csv_file = file
			file_name = the_list_dir_path+mean_std_csv_file
			list_mean_time = []
			list_std_time = []
			list_post_time = []
			with open(file_name) as csv_file:
				csv_reader = csv.reader(csv_file, delimiter=' ')
				csv_reader = list(csv_reader)
				#print csv_reader[meas_time_secs-60+1]
				for tv in time_variant:
					mean = float(csv_reader[tv][1])
					std = float(csv_reader[tv][2])	
					#print 'mean:',mean
					#print 'std:',std
					#if(std > 0):
					likelihood=calculate_likelihood(concentration,mean,std)
					#print 'likelihood:',likelihood
					posterior[pos_idx] = likelihood*posterior[pos_idx]
					#else:
					#print std, mean
					#likelihood=calculate_likelihood(concentration,mean,std)
					#print likelihood
					#posterior[pos_idx] = likelihood*posterior[pos_idx]
					if(likelihood > 1):
						print likelihood,mean,std,concentration
						print 'room:',pos_idx/len(time_variant)
						print 'time:',pos_idx%len(time_variant)
						#likelihood = 1.0
					list_likelihood.append(likelihood)
					#else:
					#	list_likelihood.append(0.0) #dummy
					#print 'posterior-',[pos_idx],':',posterior[pos_idx]
					pos_idx = pos_idx+1	
	list_likelihood_temp = list_likelihood[:]
	max_post_t = np.argmax(list_likelihood_temp)
	list_max_likelihood.append(list_likelihood_temp[max_post_t])
	#printv(verbose,"Max 1 likelihood source in room-"+str(room_source[max_post_t/len(time_variant)])+" at time-"+str(time_variant[max_post_t%len(time_variant)])+'->'+str(list_likelihood_temp[max_post_t]))

	list_likelihood_temp[max_post_t] = -999
	max_post_t = np.argmax(list_likelihood_temp)
	#printv(verbose,"Max 2 likelihood source in room-"+str(room_source[max_post_t/len(time_variant)])+" at time-"+str(time_variant[max_post_t%len(time_variant)])+'->'+str(list_likelihood_temp[max_post_t]))
		
	list_likelihood_temp[max_post_t] = -999
	max_post_t = np.argmax(list_likelihood_temp)
	#printv(verbose,"Max 3 likelihood source in room-"+str(room_source[max_post_t/len(time_variant)])+" at time-"+str(time_variant[max_post_t%len(time_variant)])+'->'+str(list_likelihood_temp[max_post_t]))
	

	posterior=normalize(posterior)
	max_post_idx = np.argmax(posterior)
	list_max_posterior_room.append(room_source[max_post_idx / len(time_variant)])

	posterior_temp = posterior[:]
	del posterior_temp[max_post_idx]
	max_post_idx = np.argmax(posterior_temp)
	list_max_posterior_room2.append(room_source[max_post_idx / len(time_variant)])
	
	del posterior_temp[max_post_idx]
	max_post_idx = np.argmax(posterior_temp)
	list_max_posterior_room3.append(room_source[max_post_idx / len(time_variant)])

	del posterior_temp[max_post_idx]
	max_post_idx = np.argmax(posterior_temp)
	list_max_posterior_room4.append(room_source[max_post_idx / len(time_variant)])

	del posterior_temp[max_post_idx]
	max_post_idx = np.argmax(posterior_temp)
	list_max_posterior_room5.append(room_source[max_post_idx / len(time_variant)])

	del posterior_temp[max_post_idx]
	max_post_idx = np.argmax(posterior_temp)
	list_max_posterior_room6.append(room_source[max_post_idx / len(time_variant)])

	del posterior_temp[max_post_idx]
	max_post_idx = np.argmax(posterior_temp)
	list_max_posterior_room7.append(room_source[max_post_idx / len(time_variant)])

	del posterior_temp[max_post_idx]
	max_post_idx = np.argmax(posterior_temp)
	list_max_posterior_room8.append(room_source[max_post_idx / len(time_variant)])

	del posterior_temp[max_post_idx]
	max_post_idx = np.argmax(posterior_temp)
	list_max_posterior_room9.append(room_source[max_post_idx / len(time_variant)])

	del posterior_temp[max_post_idx]
	max_post_idx = np.argmax(posterior_temp)
	list_max_posterior_room10.append(room_source[max_post_idx / len(time_variant)])
	#print 'pos',i,'posterior',posterior
	#print 'belief',bel
	for j in range(len(room_source)*len(time_variant)):
		list_posterior[j].append(posterior[j])

	meas_time_secs_float = meas_time_secs_float+secs_per_point
	meas_time_secs = meas_time_secs+secs_per_point
	list_time.append(meas_time_secs_float)



#print "LP:",list_posterior

fig, ax = plt.subplots()
the_max = np.argmax(posterior)
print the_max
print the_max/len(time_variant)
print the_max%len(time_variant)
print "Probability source in room-",room_source[the_max/len(time_variant)]
print "Probability source at time-",time_variant[the_max%len(time_variant)]
the_time = source_at_room[0]
a = (the_time-1)*len(time_variant)+5
b = a+len(time_variant)-5
avg_posterior = []
for i in range(a,b):
	ax.plot(list_time,list_posterior[i])

for lt in range(len(list_time)):
	temp_list = []
	for i in range(a,b):
		temp_list.append(list_posterior[i][lt])
	avg_posterior.append(np.mean(temp_list))
#ax.plot(list_time,list_posterior[the_max])
#ax.plot(list_time,list_posterior[source_at_room[0]-1])
plt.title('Probability source in room '+str(source_at_room[0])+' ('+str(source_at_room[1])+','+str(source_at_room[2])+')')
plt.xlabel('sec')
plt.ylabel('P')
legend_p = list(np.arange(6,16))
for lp in range(len(legend_p)):
	legend_p[lp]='min-'+str(legend_p[lp])

plt.legend(legend_p)
plt.show()
plt.close()

fig, ax = plt.subplots()
ax.plot(list_time[1:],list_max_likelihood)
#ax.plot(list_time,list_posterior[the_max])
#ax.plot(list_time,list_posterior[source_at_room[0]-1])
plt.title('Max likelihood source in room '+str(source_at_room[0])+' ('+str(source_at_room[1])+','+str(source_at_room[2])+')')
plt.xlabel('sec')
plt.ylabel('P')
#plt.legend([11,12,13,14,15])
plt.show()
plt.close()



fig, ax = plt.subplots()
ax.plot(list_time[1:],list_max_posterior_room)
#ax.plot(list_time[1:],list_max_posterior_room2)
#ax.plot(list_time[1:],list_max_posterior_room3)
#ax.plot(list_time[1:],list_max_posterior_room4)
#ax.plot(list_time[1:],list_max_posterior_room5)
#ax.plot(list_time[1:],list_max_posterior_room6)
#ax.plot(list_time[1:],list_max_posterior_room7)
#ax.plot(list_time[1:],list_max_posterior_room8)
#ax.plot(list_time[1:],list_max_posterior_room9)
#ax.plot(list_time[1:],list_max_posterior_room10)
#ax.plot(list_time,list_posterior[the_max])
#ax.plot(list_time,list_posterior[source_at_room[0]-1])
plt.title('Room max posterior, source in room '+str(source_at_room[0])+' ('+str(source_at_room[1])+','+str(source_at_room[2])+')')
plt.xlabel('sec')
plt.ylabel('room-')
plt.grid(True)
plt.yticks(list(np.arange(1,len(room_source),1)))
plt.ylim(0,len(room_source)+1)
#plt.legend([1,2,3,4,5,6,7,8,9,10])
plt.show()
plt.close()


# create map image
file_name = current_folder+'/OccupancyGrid3Droom.csv'
with open(file_name) as csv_file:
	csv_reader = csv.reader(csv_file, delimiter=',')
	for row in csv_reader:
		b_list = []
		#del row[-1]
		for col in row:
			#print col
			b_list.append(float(col))
		a_list.append(b_list)
              
#print(f'Processed {line_count} lines.')
#print a_list
the_map = np.array(a_list)
fig, ax = plt.subplots()
im = ax.imshow((np.rot90(the_map)),cmap="gist_ncar")
meas_pos_x = []
meas_pos_y = []
n = []
print the_map.shape
j=0
for i in meas_pos:
	meas_pos_x.append(i[0]/cell_size)
	meas_pos_y.append(the_map.shape[1]-i[1]/cell_size)
	n.append(j)
	j=j+1
#del list_posterior[0][0]
#del list_posterior[b][0]

ax.scatter(meas_pos_x,meas_pos_y,cmap = 'jet',c=avg_posterior[1:])#list_posterior[b])
ax.scatter([source_at_room[1]/cell_size],[the_map.shape[1]-source_at_room[2]/cell_size],c='white')
#for i, txt in enumerate(n):
#    ax.annotate(txt, (meas_pos_x[i], meas_pos_y[i]))

plt.title('Gas source in room '+str(source_at_room[0])+' ('+str(source_at_room[1])+','+str(source_at_room[2])+')')
plt.show()

plt.close()

#exit()
	