#!/usr/bin/env python3

import numpy
import random
import sys
import math

import geometry_msgs.msg
import moveit_msgs.msg
import moveit_msgs.srv
import rospy
import tf
import moveit_commander
from urdf_parser_py.urdf import URDF
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Transform
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def convert_to_message(T):
    t = geometry_msgs.msg.Pose()
    position = tf.transformations.translation_from_matrix(T)
    orientation = tf.transformations.quaternion_from_matrix(T)
    t.position.x = position[0]
    t.position.y = position[1]
    t.position.z = position[2]
    t.orientation.x = orientation[0]
    t.orientation.y = orientation[1]
    t.orientation.z = orientation[2]
    t.orientation.w = orientation[3]        
    return t

class MoveArm(object):

    def __init__(self):

        #Loads the robot model, which contains the robot's kinematics information
        self.num_joints = 0
        self.joint_names = []
        self.joint_axes = []
        self.robot = URDF.from_parameter_server()
        self.base = self.robot.get_root()
        self.get_joint_info()

        # Wait for moveit IK service
        rospy.wait_for_service("compute_ik")
        self.ik_service = rospy.ServiceProxy('compute_ik',  moveit_msgs.srv.GetPositionIK)
        print("IK service ready")

        # Wait for validity check service
        rospy.wait_for_service("check_state_validity")
        self.state_valid_service = rospy.ServiceProxy('check_state_validity',  
                                                      moveit_msgs.srv.GetStateValidity)
        print("State validity service ready")

        # MoveIt parameter
        robot_moveit = moveit_commander.RobotCommander()
        self.group_name = robot_moveit.get_group_names()[0]

        #Subscribe to topics
        rospy.Subscriber('/joint_states', JointState, self.get_joint_state)
        rospy.Subscriber('/motion_planning_goal', Transform, self.motion_planning)
        self.current_obstacle = "None"
        rospy.Subscriber('/obstacle', String, self.get_obstacle)

        #Set up publisher
        self.pub = rospy.Publisher('/joint_trajectory', JointTrajectory, queue_size=10)

    '''This callback provides you with the current joint positions of the robot 
     in member variable q_current.'''
    def get_joint_state(self, msg):
        self.q_current = []
        for name in self.joint_names:
            self.q_current.append(msg.position[msg.name.index(name)])

    '''This callback provides you with the name of the current obstacle which
    exists in the RVIZ environment. Options are "None", "Simple", "Hard",
    or "Super". '''
    def get_obstacle(self, msg):
        self.current_obstacle = msg.data

    '''This is the callback which will implement your RRT motion planning.
    You are welcome to add other functions to this class (i.e. an
    "is_segment_valid" function will likely come in handy multiple times 
    in the motion planning process and it will be easiest to make this a 
    seperate function and then call it from motion planning). You may also
    create trajectory shortcut and trajectory sample functions if you wish, 
    which will also be called from the motion planning function.'''    
    def motion_planning(self, ee_goal):
        print("Starting motion planning")
    ########INSERT YOUR RRT MOTION PLANNING HERE##########
        q_ee_curr = numpy.array(self.q_current)
        q_init = q_ee_curr
        
        RRTree = []
        RRTbranch_list = []
        RRTree.append(q_ee_curr)
        branchLength = 0.1
    
        Trans = tf.transformations.translation_matrix(numpy.array([ee_goal.translation.x, ee_goal.translation.y, ee_goal.translation.z]))
	
        Rot = tf.transformations.quaternion_matrix(numpy.array([ee_goal.rotation.x, ee_goal.rotation.y, ee_goal.rotation.z, ee_goal.rotation.w]))
	
	
        b_T_ee_goal = numpy.dot(Trans,Rot)
	
        #q_goal_list = self.IK(b_T_ee_goal)
	
        q_ee_goal = numpy.array(self.IK(b_T_ee_goal))
	# Make Tree and Branch List
	
        while (self.is_segment_valid(q_init, q_ee_goal) == False):
              pi = math.pi
              q_rand = numpy.zeros(self.num_joints)
              for i in range(len(q_rand)):
	           
                  q_rand[i] = 2*pi*numpy.random.rand(1) - pi # Ranges from 0(-pi) to 2pi(-pi) [-pi to pi]
	          
	          
              init_length = 1e8
	     #Branches out tree from the nearest node to the random node in the direction of the random node
              for i in range(len(RRTree)):
	          
                  curr_dist = numpy.linalg.norm(q_rand - RRTree[i])
                  if (curr_dist <= init_length):
                     q_nearest = RRTree[i]
                     init_length = curr_dist
	             
              q_next = q_nearest + branchLength * ((q_rand - q_nearest)/numpy.linalg.norm(q_rand - q_nearest))
	       #Ensures the new branch doesn't hit obstacles
              if self.is_segment_valid(q_nearest, q_next) == True:
	          
                  q_init = q_next
                  RRTree.append(q_next)
                  RRTbranch_list.append(RRTBranch(q_nearest, q_next))
                  
              else:
              
                   q_init = q_ee_curr
	              
	
        RRTbranch_list.append(RRTBranch(q_init, q_ee_goal))
        RRTree.append(q_ee_goal)   
	# Make initial trajectory from current pose to goal pose
        initial_path = []
        initial_path.append(q_ee_goal)
        index = 1
        currRRTbranch = RRTbranch_list[-index].q
        #Note_to_self
        #Cannot compare two vectors (Must compare norms of vectors AVOID ERROR)
        while (numpy.linalg.norm(currRRTbranch) != numpy.linalg.norm(q_ee_curr)):
             while (numpy.linalg.norm(RRTbranch_list[-index].q) != numpy.linalg.norm(currRRTbranch)):
                   index +=1
             currRRTbranch = RRTbranch_list[-index].parent
             initial_path[0:0] = [currRRTbranch]
        initial_path[0:0] = [q_ee_curr]
	
	
	#Shorten Trajectory
	
        
        curr_start = initial_path[0]
        short_path = []
        short_path.append(q_ee_curr)
       
        
        for i in range(1,len(initial_path)):
        
            if(self.is_segment_valid(curr_start,initial_path[i])==False):
            
               
            
               short_path.append(initial_path[i-1])
               curr_start = initial_path[i-1]
               i = i-1
        
        short_path.append(initial_path[-1])
        
        #print(short_path)
        
        
        #Failed Implementation of Shortening Path
        #index2 = 0
        #while (numpy.linalg.norm(initial_path[index2]) != numpy.linalg.norm(q_ee_goal)):
	      
          #    while(self.is_segment_valid(curr_start, initial_path[index2]) == True):
	   #   
            #       if(numpy.linalg.norm(initial_path[index2]) == numpy.linalg.norm(q_ee_goal)):
	           
             #         break
	     
              #     index2+=1
              #if(numpy.linalg.norm(initial_path[index2]) == numpy.linalg.norm(q_ee_goal)):
	           
               #       break
	           
             
	            
              #short_path.append(initial_path[index2-1])
              #curr_start = initial_path[index2-1]   
	      
        #short_path.append(q_ee_goal)
	
	# Resample shortened trajectory
	
        final_path = []
        maxseglength = 0.5
        final_path.append(q_ee_curr)
	
        for i in range(1,len(short_path)):
	
            delq = short_path[i] - short_path[i-1]
            currdist = numpy.linalg.norm(delq)
            numseg = math.ceil(currdist/maxseglength)
	    
            for k in range(numseg+1):
	       
               newq = short_path[i-1] + maxseglength*k*delq/currdist
               final_path.append(newq)
	        
        final_path.append(q_ee_goal)
	
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        for n in range(len(final_path)):
	    
            currPoint = JointTrajectoryPoint()
            currPoint.positions = final_path[n]
            trajectory.points.append(currPoint)
	    
        self.pub.publish(trajectory)
		
	
        ######################################################

    """ This function will perform IK for a given transform T of the end-effector.
    It returns a list q[] of values, which are the result positions for the 
    joints of the robot arm, ordered from proximal to distal. If no IK solution 
    is found, it returns an empy list.
    """
    def IK(self, T_goal):
        req = moveit_msgs.srv.GetPositionIKRequest()
        req.ik_request.group_name = self.group_name
        req.ik_request.robot_state = moveit_msgs.msg.RobotState()
        req.ik_request.robot_state.joint_state.name = self.joint_names
        req.ik_request.robot_state.joint_state.position = numpy.zeros(self.num_joints)
        req.ik_request.robot_state.joint_state.velocity = numpy.zeros(self.num_joints)
        req.ik_request.robot_state.joint_state.effort = numpy.zeros(self.num_joints)
        req.ik_request.robot_state.joint_state.header.stamp = rospy.get_rostime()
        req.ik_request.avoid_collisions = True
        req.ik_request.pose_stamped = geometry_msgs.msg.PoseStamped()
        req.ik_request.pose_stamped.header.frame_id = self.base
        req.ik_request.pose_stamped.header.stamp = rospy.get_rostime()
        req.ik_request.pose_stamped.pose = convert_to_message(T_goal)
        req.ik_request.timeout = rospy.Duration(3.0)
        res = self.ik_service(req)
        q = []
        if res.error_code.val == res.error_code.SUCCESS:
            q = res.solution.joint_state.position
        return q

    '''This is a function which will collect information about the robot which
       has been loaded from the parameter server. It will populate the variables
       self.num_joints (the number of joints), self.joint_names and
       self.joint_axes (the axes around which the joints rotate)'''
    def get_joint_info(self):
        link = self.robot.get_root()
        while True:
            if link not in self.robot.child_map: break
            (joint_name, next_link) = self.robot.child_map[link][0]
            current_joint = self.robot.joint_map[joint_name]
            if current_joint.type != 'fixed':
                self.num_joints = self.num_joints + 1
                self.joint_names.append(current_joint.name)
                self.joint_axes.append(current_joint.axis)
            link = next_link


    """ This function checks if a set of joint angles q[] creates a valid state,
    or one that is free of collisions. The values in q[] are assumed to be values
    for the joints of the KUKA arm, ordered from proximal to distal. 
    """
    def is_state_valid(self, q):
        req = moveit_msgs.srv.GetStateValidityRequest()
        req.group_name = self.group_name
        req.robot_state = moveit_msgs.msg.RobotState()
        req.robot_state.joint_state.name = self.joint_names
        req.robot_state.joint_state.position = q
        req.robot_state.joint_state.velocity = numpy.zeros(self.num_joints)
        req.robot_state.joint_state.effort = numpy.zeros(self.num_joints)
        req.robot_state.joint_state.header.stamp = rospy.get_rostime()
        res = self.state_valid_service(req)
        return res.valid
        
    def is_segment_valid(self, q_ee_curr, q_ee_goal):
    	
    	segLength = 0.3
    	#print(q_ee_goal)
    	#print(q_ee_curr)
    	deltaq = q_ee_goal - q_ee_curr
    	
    	#Find the distance between the two configurations in joint space
    	
    	dist = numpy.linalg.norm(deltaq)
    	
    	if dist == 0:
    	   return True
    	   
    	segNum = int(dist/segLength)
    	unit_deltaq = deltaq/dist
    	for i in range(segNum+1):
    	
    	    q_next = q_ee_curr + segLength*i*unit_deltaq
    	    if (self.is_state_valid(q_next) == False):
    	    
    	       return False
    	       
    	return True
    	
'''This is a class which you can use to keep track of your tree branches.
It is easiest to do this by appending instances of this class to a list 
(your 'tree'). The class has a parent field and a joint position field (q). 
You can initialize a new branch like this:
RRTBranch(parent, q)
Feel free to keep track of your branches in whatever way you want - this
is just one of many options available to you.'''
class RRTBranch(object):
    def __init__(self, parent, q):
        self.parent = parent
        self.q = q


if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_arm', anonymous=True)
    ma = MoveArm()
    rospy.spin()

