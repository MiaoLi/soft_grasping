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
from multiprocessing import Process

from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Transform, Point, TwistStamped



from numpy import *



JointTarget = JointState()
JointCurrent = JointState()

KUKACartMsg = PoseStamped()

def Joint_PD():
    # Joint PD control for allegro hand
    pub = rospy.Publisher('/allegroHand_0/joint_cmd', JointState)

    rospy.Subscriber('/planned_grasp/finger_joints', JointState, Joint_TargetCallBack)
    rospy.Subscriber('/allegroHand_0/joint_states', JointState, Joint_CurrentCallBack)
    

    rospy.init_node('HandJoint', anonymous=True)  
    
    r = rospy.Rate(200) # 10hz
    print('run Hand controller-----------')
    
    JointCmd = JointState()

    Joints.name = ''.join("Allegro")    
    while not rospy.is_shutdown():
        #'header', 'name', 'position', 'velocity', 'effort'
        #rospy.loginfo(Joints)  
        Joints.velocity = ''
        Joints.effort = ''
        pub.publish(Joints)
        r.sleep()   
   
def Joint_CurrentCallBack(msg):
	global JointCurrent
	# del Joints.position[:]
	for i in range(16):
		JointCurrent.position.append(msg.position[i])

def Joint_TargetCallBack(msg):
    global JointTarget
    # del Joints.position[:]
    for i in range(16):
        JointTarget.position.append(msg.position[i])


def Finger_Closing(VecCurrent, VecTarget, KGain):
    #KGain is the gain vector for each finger
    tmp= VecTarget - VecCurrent
    for i in range(4):
        VecTarget[4*i]


def KUKA_CartImp():

    # send the cartesian impedance command to kuka 

    pub = rospy.Publisher('/KUKA/des_ee_pose', PoseStamped)
    rospy.Subscriber('/KUKA/Pose', PoseStamped, KUKA_cartCallBack)

    rospy.init_node('KUKAPosition', anonymous =True)  
    r = rospy.Rate(100) 
    
    print('Run robot  controller------------')

    while linalg.norm(array([KUKACartMsg.translation.x,KUKACartMsg.translation.y,KUKACartMsg.translation.z]))==0:  
        try:
            print("Waiting for the /KUKA/Pose topic")
        except (KeyboardInterrupt, SystemExit):
            print "Exiting..."
        sleep(0.01)

    while not rospy.is_shutdown():
        if linalg.norm(array([KUKACartMsg.position.x,KUKACartMsg.position.y,KUKACartMsg.position.z]))!=0:
            rospy.loginfo(KUKACartMsg)
            pub.publish(KUKACartMsg)
            r.sleep()


def KUKA_cartCallBack(msg):
    # here change the msg format
    global KUKACartMsg  
    KUKACartMsg.pose.orientation.x = msg.pose.orientation.x 
    KUKACartMsg.pose.orientation.y = msg.pose.orientation.y
    KUKACartMsg.pose.orientation.z = msg.pose.orientation.z
    KUKACartMsg.pose.orientation.w = msg.pose.orientation.w

    KUKACartMsg.position.x = msg.position.x
    KUKACartMsg.position.y = msg.position.y
    KUKACartMsg.position.z = msg.position.z

if __name__ == '__main__':
    try:
        p1 = Process(target = Joint_PD)
        p1.start()        
        p2 = Process(target = KUKA_CartImp)
        p2.start()

    except rospy.ROSInterruptException: pass