#!/usr/bin/python

"""
This function is used to close finger compliantly given the tactile and position uncertainties. 
It was modified from KUKAMotion and KukaReaching package, and the AllegroLib package;

Now we use the 'kuka_interface_packages' for arm control,
 and use the 'allegro-hand-ros' package for the hand control in PD mode;

"""

import rospy
import roslib
from time import *

from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Transform, Point

from multiprocessing import Process
from numpy import *



Joints= JointState()
KUKACartMsg= Transform()

def Joint_PD():
    # Joint position control
    pub = rospy.Publisher('/allegro/joint_cmd', JointState)
    rospy.Subscriber("allegroJointState", AllegroJointState, Joint_PDCallBack)
    rospy.init_node('HandJoint', anonymous=True)  
    r = rospy.Rate(200) # 10hz
    print('run Hand controller-----------')
    
    Joints.name = ''.join("Allegro")
    while not rospy.is_shutdown():
        #'header', 'name', 'position', 'velocity', 'effort'
        #rospy.loginfo(Joints)  
        Joints.velocity = ''
        Joints.effort = ''
        pub.publish(Joints)
        r.sleep()   
   
        

def Joint_PDCallBack(msg):
	global Joints
	del Joints.position[:]
	for i in range(16):
		Joints.position.append(msg.joints[i])

def KUKA_Cart():
    # send the cartesian command to kuka
    pub = rospy.Publisher("/KUKAMotion/CartPoseCmd", Transform)
    rospy.Subscriber("/kukaArmState", Transform, KUKA_cartCallBack)
    rospy.init_node('KUKAPosition', anonymous =True)  
    r = rospy.Rate(100) 
    
    print('Run robot  controller------------')

    while linalg.norm(array([KUKACartMsg.translation.x,KUKACartMsg.translation.y,KUKACartMsg.translation.z]))==0:  
        try:
            print("Waiting for the KUKACartPose topic")
        except (KeyboardInterrupt, SystemExit):
            print "Exiting..."
        sleep(0.01)

    while not rospy.is_shutdown():
        if linalg.norm(array([KUKACartMsg.translation.x,KUKACartMsg.translation.y,KUKACartMsg.translation.z]))!=0:
            rospy.loginfo(KUKACartMsg)
            pub.publish(KUKACartMsg)
            r.sleep()

def KUKA_cartCallBack(msg):
    # here change the msg format
    global KUKACartMsg
    KUKACartMsg.rotation.w = msg.rotation.w
    KUKACartMsg.rotation.x = msg.rotation.x 
    KUKACartMsg.rotation.y = msg.rotation.y
    KUKACartMsg.rotation.z = msg.rotation.z
    KUKACartMsg.translation.x = msg.translation.x
    KUKACartMsg.translation.y = msg.translation.y
    KUKACartMsg.translation.z = msg.translation.z


if __name__ == '__main__':
    try:
        p1 = Process(target = Joint_PD)
        p1.start()        
        p2 = Process(target = KUKA_Cart)
        p2.start()

    except rospy.ROSInterruptException: pass