
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

import rospy
import numpy as np

def HandControl(Joints, Speed, PublisherName):
	#Script to control the hand 
	#'j33_joint' -> F3-Fingertip
	#'j32_joint' -> F3-Base
	#
	#'j11_joint' -> F1-Spread
	#'j12_joint' -> F1-Base
	#'j13_joint' -> F1-Fingertip
	#
	#'j21_joint' -> F2-Spread
	#'j22_joint' -> F2-Base
	#'j23_joint' -> F2-Fingertip


def ArmControl(command, PublisherName):
	#Script to control the KUKA arm
	# std_msgs/Header header
	# string[] name
	# float64[] position
	# float64[] velocity
	# float64[] effort
	# float64[] stiffness


def DataRecord():
	# start the data record
	HandPublisher = rospy.Publisher('/bhand_node/command', JointState, queue_size=1)
	ArmPublisher =  rospy.Publisher('kuka_fri_bridge/JointStateImpedance',JointStateImpedance, queue_size=1)
	HandJoints = np.array([0.1,0.1,0.1,0.1])
	HandSpeed = np.array([0.1,0.1,0.1,0.1])

	ArmJointImpCmd = JointStateImpedance()
	ArmJointImpCmd.name = 'LWR'
	ArmJointImpCmd.position = ''
	ArmJointImpCmd.velocity = ''
	ArmJointImpCmd
	



if __name__ == '__main__':
	try:
		DataRecord()
	except rospy.ROSInterruptException: pass

