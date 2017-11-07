#!/usr/bin/env python

import rospy
from trac_ik_baxter.srv import *
from geometry_msgs.msg import PoseStamped
import numpy as np
from sensor_msgs.msg import JointState

file_name = "/home/tony/dmpbbo/dmpbbo_baxter/baxter_raw_data/cartesion_baxter_data.txt"
row_data = np.loadtxt(file_name)
data_length = len(row_data)
resample_t = np.linspace(row_data[0,0],row_data[-1,0],data_length)
position_x = np.interp(resample_t, row_data[:,0],row_data[:,1])
position_y = np.interp(resample_t, row_data[:,0],row_data[:,2])
position_z = np.interp(resample_t, row_data[:,0],row_data[:,3])
orientation_x = np.interp(resample_t, row_data[:,0],row_data[:,4])
orientation_y = np.interp(resample_t, row_data[:,0],row_data[:,5])
orientation_z = np.interp(resample_t, row_data[:,0],row_data[:,6])
orientation_w = np.interp(resample_t, row_data[:,0],row_data[:,7])

rospy.init_node('get_ik')
rospy.wait_for_service('trac_ik_right')
getik = rospy.ServiceProxy('trac_ik_right', GetConstrainedPositionIK)
print "Ready to get ik"
ik_request = GetConstrainedPositionIKRequest()
ik_response = GetConstrainedPositionIKResponse()
point = PoseStamped()
joint_state = JointState()

for i in range(data_length):
    point.pose.position.x = position_x[i]
    point.pose.position.y = position_y[i]
    point.pose.position.z = position_z[i]
    point.pose.orientation.x = orientation_x[i]
    point.pose.orientation.y = orientation_y[i]
    point.pose.orientation.z = orientation_z[i]
    point.pose.orientation.w = orientation_w[i]
    point.header.stamp.nsecs = resample_t[i]
    ik_request.pose_stamp.append(point)
ik_request.num_steps = 100
ik_request.end_tolerance = 0.01
ik_response = getik(ik_request)
print ik_response.joints
rospy.loginfo("get ik")

