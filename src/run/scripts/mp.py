#!/usr/bin/env python

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
        print "IK service ready"

        # Wait for validity check service
        rospy.wait_for_service("check_state_validity")
        self.state_valid_service = rospy.ServiceProxy('check_state_validity',  
                                                      moveit_msgs.srv.GetStateValidity)
        print "State validity service ready"

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
        print "Starting motion planning"
	########INSERT YOUR RRT MOTION PLANNING HERE##########
        t_goal = tf.transformations.translation_matrix((ee_goal.translation.x, ee_goal.translation.y, ee_goal.translation.z))
        r_goal = tf.transformations.quaternion_matrix((ee_goal.rotation.x, ee_goal.rotation.y, ee_goal.rotation.z, ee_goal.rotation.w))
        T_goal = numpy.dot(t_goal, r_goal)
        q_goal = numpy.array(self.IK(T_goal))
        tree = []
        index = []
        tree.append(self.q_current)
        index.append(0)
        tree2 = []
        index2 = []
        tree2.append(q_goal)
        index2.append(0)

        
        #when no new point needed
        samplepoint = self.trajectory_sample(self.q_current, q_goal)
        flag = 0
        for i in range(len(samplepoint)):
            if self.is_state_valid(samplepoint[i]) == False:
                flag = 1
        if flag == 0:
            path = []
            path.append(self.q_current)
            path.append(q_goal)


        else:
            count = 1
            restart_num = 30
            ik_times = 0
            while True:
                # if no soultion found, restart and try new ik
                if count > restart_num:
                    print "Restart"
                    new_q_goal = []
                    new_q_goal.append(list(q_goal))
                    while True:
                        # if no new ik found, stop trying and continue 
                        ik_times = ik_times + 1
                        if ik_times > 50:
                            restart_num = restart_num*100
                            print "no new IK found, restart rejected"
                            break
                        q_goal = numpy.array(self.IK(T_goal))
                        tree = []
                        index = []
                        tree.append(self.q_current)
                        index.append(0)
                        tree2 = []
                        index2 = []
                        tree2.append(q_goal)
                        index2.append(0)
                        if list(q_goal) not in new_q_goal:
                            new_q_goal.append(list(q_goal))
                            count = 1
                            break

                print 'Caculating:', count
                count = count + 1
                # generate random point
                rand = []
                for num in range(self.num_joints):
                    rand.append(random.uniform(-math.pi, math.pi))
                rand = numpy.array(rand)

                #find closest tree point
                lengthlist = []
                for i in range(len(tree)):
                    length = numpy.linalg.norm( tree[i] - rand )
                    lengthlist.append(length)
                minindex = lengthlist.index(min(lengthlist))

                #add point to the tree
                if min(lengthlist) > 0.3:
                    ori = (rand - tree[minindex])/min(lengthlist)
                    newpoint = tree[minindex] + 1*ori
                    samplepoint = self.trajectory_sample(tree[minindex], newpoint)
                    for i in range(len(samplepoint)):
                        if self.is_state_valid(samplepoint[i]) == True:
                            newpoint = samplepoint[i]
                        else:
                            newpoint = samplepoint[i-1]
                            break  
                else:
                    if self.is_state_valid(rand) == True:
                        newpoint = rand
                    else:
                        continue
                tree.append(newpoint)
                index.append(minindex)

                #same random point
                rand2 = rand

                #find closest tree point
                lengthlist2 = []
                for i in range(len(tree2)):
                    length2 = numpy.linalg.norm( tree2[i] - rand2 )
                    lengthlist2.append(length2)
                minindex2 = lengthlist2.index(min(lengthlist2))

                #add point to the tree
                if min(lengthlist2) > 0.3:
                    ori2 = (rand2 - tree2[minindex2])/min(lengthlist2)
                    newpoint2 = tree2[minindex2] + 1*ori2
                    samplepoint2 = self.trajectory_sample(tree2[minindex2], newpoint2)
                    for i in range(len(samplepoint2)):
                        if self.is_state_valid(samplepoint2[i]) == True:
                            newpoint2 = samplepoint2[i]
                        else:
                            newpoint2 = samplepoint2[i-1]
                            break  
                else:
                    if self.is_state_valid(rand2) == True:
                        newpoint2 = rand2
                    else:
                        continue
                tree2.append(newpoint2)
                index2.append(minindex2)

                #see if there is no obstacle between 2 newpoint
                flag1 = 0
                flag = 0
                samplepoint = self.trajectory_sample(newpoint, newpoint2)
                for j in range(len(samplepoint)):
                    if self.is_state_valid(samplepoint[j]) == False:
                        flag = 1
                if flag == 0:
                    flag1 = 1
                    break

                #see if there is no obstacle between newpoints and random point
                flag = 0
                samplepoint = self.trajectory_sample(newpoint, rand)
                for j in range(len(samplepoint)):
                    if self.is_state_valid(samplepoint[j]) == False:
                        flag = 1
                samplepoint = self.trajectory_sample(newpoint2, rand)
                for j in range(len(samplepoint)):
                    if self.is_state_valid(samplepoint[j]) == False:
                        flag = 1       
                if flag == 0:
                    flag1 = 2
                    break

            #get the path from the tree
            if flag1 == 1:
                path = []
                path.append(newpoint)
                i = index[len(index)-1]
                while True:
                    path.append(tree[i])
                    i=index[i]
                    if i == 0:
                        break
                path.append(numpy.array(self.q_current))
                path.reverse()

                path2 = []
                path2.append(newpoint2)
                i = index2[len(index2)-1]
                while True:
                    path2.append(tree2[i])
                    i=index2[i]
                    if i == 0:
                        break
                path2.append(numpy.array(q_goal))
                path = path + path2

            if flag1 == 2:
                path = []
                path.append(rand)
                path.append(newpoint)
                i = index[len(index)-1]
                while True:
                    path.append(tree[i])
                    i=index[i]
                    if i == 0:
                        break
                path.append(numpy.array(self.q_current))
                path.reverse()

                path2 = []
                path2.append(newpoint2)
                i = index2[len(index2)-1]
                while True:
                    path2.append(tree2[i])
                    i=index2[i]
                    if i == 0:
                        break
                path2.append(numpy.array(q_goal))
                path = path + path2

        #short cut
        scpath = []
        scpath.append(path[0])
        start = path[0]
        for i in range(len(path)-1):
            point = self.trajectory_sample(start, path[i+1])
            flag = 0
            for j in range(len(point)):
                if self.is_state_valid(point[j]) == False:
                    flag = 1
            if flag == 1:
                scpath.append(path[i])
                start = path[i]
        scpath.append(q_goal)
        path = scpath


        #sample final path
        finalpath = []
        finalpath.append(path[0])
        for i in range(len(path)-1):
            samplepoint = self.trajectory_sample(path[i], path[i+1])
            samplepoint.pop(0)
            finalpath = finalpath + samplepoint


        joint_trajectory = JointTrajectory()
        for i in range(len(finalpath)):
            point = JointTrajectoryPoint()
            point.positions = finalpath[i]
            joint_trajectory.points.append(point)
        joint_trajectory.joint_names = self.joint_names
        self.pub.publish(joint_trajectory)
        ######################################################
    

    def trajectory_sample(self, start, end):
        length = numpy.linalg.norm(start - end)
        num = math.ceil(length/0.2)
        num = int(num)
        unit = (end - start)/num
        point = []
        point.append(start)
        for i in range(num-1):
            point.append(start + (i + 1)*unit)
        point.append(end)
        return point




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

