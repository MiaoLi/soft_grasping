
"""

This script is used to record the data during grasp-on-table demonstration.

It requires the following packages:
	--- syntouchpublisher
	--- kuka_interface_packages
	--- barrett_hand

The data collection procedure is as follows:

data: { tactile + finger close speed + arm wrist twist }

(1) We run the kuka robot in gravity compensation mode using kuka_interface_packages

(2) We close the fingers of Barrett hand with given speed.

(3) Human can only adapt the arm to grasp the objects on table. 
	The arm will change back to postion mode and the finger will stop closing when 
	the demonstrator consider the grasp is finshed, which can be either successed or failed.
 
(4) What varies: The tilt angle of the table support, the material of the surface,
	 and the finger closing speed.

""" 

import roslib 
roslib.load_manifest('syntouchpublisher') 
roslib.load_manifest('bhand_controller')
roslib.load_manifest('kuka_fri_bridge')

from kuka_fri_bridge.msg import JointStateImpedance
from sensor_msgs.msg import JointState
from std_msgs.msg import Header, String

import rospy
import numpy as np
from numpy import linalg as LA

def HandControl(Joints, Speed, PublisherName):
	#Script to control the hand 
	#'j33_joint' -> F3-Fingertip
	#'j32_joint' -> F3-Base    *****
	#
	#'j11_joint' -> F1-Spread  ****
	#'j12_joint' -> F1-Base    ****
	#'j13_joint' -> F1-Fingertip
	#
	#'j21_joint' -> F2-Spread  *****
	#'j22_joint' -> F2-Base    *****
	#'j23_joint' -> F2-Fingertip
    #'joint_ids': [ 'F1', 'F1_TIP', 'F2', 'F2_TIP', 'F3', 'F3_TIP', 'SPREAD_1', 'SPREAD_2'],
    #'joint_names': ['bh_j12_joint', 'bh_j13_joint', 'bh_j22_joint', 'bh_j23_joint', 'bh_j32_joint', 'bh_j31_joint', 'bh_j11_joint', 'bh_j21_joint']
    return 0
def ArmControl(command, PublisherName):
	#Script to control the KUKA arm
	# std_msgs/Header header
	# string[] name
	# float64[] position
	# float64[] velocity
	# float64[] effort
	# float64[] stiffness
	return 0
def GetUsrCmd(msg):
	global UsrCmd
	UsrCmd = msg

def DataRecord():
	
	rospy.init_node('data_record', anonymous=True)
	r = rospy.Rate(10) #change this to any feasible rate

	# start the data record
	HandPublisher = rospy.Publisher('/bhand_node/command', JointState, queue_size=1)
	ArmPublisher =  rospy.Publisher('kuka_fri_bridge/JointStateImpedance',JointStateImpedance, queue_size=1)
	rospy.Subscriber('/usr/cmd',String, GetUsrCmd, queue_size=1)
	global UsrCmd
	UsrCmd = String()
	# The hand start with fully open and close fingers gradually.
	
	HandJointsFinal = np.array([1.5,1.5,1.5,0.0])

	SpeedScale = 5
	HandSpeed = SpeedScale * np.array([0.1, 0.1, 0.1, 0.0])

	HandCmd =  JointState()
	h = Header()
	h.stamp = rospy.Time.now()
	HandCmd.header = h
	HandCmd.name = ['bh_j23_joint', 'bh_j12_joint', 'bh_j22_joint', 'bh_j32_joint', 'bh_j33_joint', 'bh_j13_joint', 'bh_j11_joint', 'bh_j21_joint']
	
	# HandCmd.position = [HandJoints[0], 0.0, HandJoints[1], 0.0, HandJoints[2], 0.0, HandJoints[3], HandJoints[3]]
	# HandCmd.velocity= [HandSpeed[0], 0.0, HandSpeed[1], 0.0, HandSpeed[2], 0.0, 0.0, 0.0]
	HandCmd.effort = ''

	# The arm is in the gravity compensation mode
	ArmJointImpCmd = JointStateImpedance()
	ArmJointImpCmd.name = 'LWR'
	ArmJointImpCmd.position = ''
	ArmJointImpCmd.velocity = ''
	ArmJointImpCmd.effort = ''
	ArmJointImpCmd.stiffness = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
	Delta_time = 0.1
	HandJoints = np.zeros([4])
	while not rospy.is_shutdown():
		h.stamp = rospy.Time.now()
		HandCmd.header = h
		# print UsrCmd.data
		# print UsrCmd.data == 'stop'
		if UsrCmd.data == 'arm':
			HandPublisher.publish(HandCmd)
		else:
			print 'Arm is in position mode'

		if UsrCmd.data != 'stop' and LA.norm(HandJoints-HandJointsFinal)>0.1:
			HandJoints = HandJoints + HandSpeed * Delta_time		
			HandCmd.position = [0.0, HandJoints[0], HandJoints[1],HandJoints[2], 0.0, 0.0,HandJoints[3], HandJoints[3]]
			HandCmd.velocity= [0.0, HandSpeed[0], HandSpeed[1], HandSpeed[2], 0.0, 0.0, HandSpeed[3], HandSpeed[3]]		
			ArmPublisher.publish(ArmJointImpCmd)
		else:
			print "stop command the hand"
		r.sleep()







if __name__ == '__main__':
	try:
		DataRecord()
	except rospy.ROSInterruptException: pass

