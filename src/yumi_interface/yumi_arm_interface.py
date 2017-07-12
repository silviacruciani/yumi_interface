#! /usr/bin/env python

"""
    This file contains a python interface to the yumi robot with velocity control.

    @author: Silvia Cruciani (cruciani@kth.se)
"""

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import PyKDL
from kdl_parser import kdl_tree_from_urdf_model
from urdf_parser_py.urdf import URDF
import numpy as np
from yumi_hw.srv import * #using this for grasping

class YumiArm(object):
    """docstring for YumiArm"""
    def __init__(self, arm_name):
        #read the desired arm (right or left)
        self._arm_name = arm_name[0]
        if self._arm_name != 'r' and self._arm_name !='l':
            rospy.logwarn("Wrong arm name input. Using default: right")
            self._arm_name = 'r'

        #initialize the ros topic publishers
        self._ros_topic_namespace = rospy.get_param('/yumi/velocity_control/ros_topic_command_name', '/yumi/joint_vel_controller_')
        self._publishers = [None, None, None, None, None, None, None]
        self.initialize_ros_publishers()

        #now initialize the clients to open and close the gripper and wait for the servers to come up
        self._open_service_name = rospy.get_param('/yumi/gripper_server_open_name', '/yumi/yumi_gripper/release_grasp')
        self._close_service_name = rospy.get_param('/yumi/gripper_server_close_name', '/yumi/yumi_gripper/do_grasp')

        rospy.loginfo('Waiting for gripper servers')
        rospy.wait_for_service(self._open_service_name, 5.0)
        rospy.wait_for_service(self._close_service_name, 5.0)

        self._open_service = rospy.ServiceProxy(self._open_service_name, YumiGrasp)
        self._close_service = rospy.ServiceProxy(self._close_service_name, YumiGrasp)


        #and the variables for the current position and gripper id
        self._gripper_position = 0.0 #in meters
        self._gripper_id = 1
        if self._arm_name == 'r':
            self._gripper_id = 2

        #initialize the joint values to 0
        self._joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self._joint_velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        #store a neutral join position
        self._joint_neutral = [-1.4, -2.1, 0.74, 0.3, 0.0, 0.0, 0.0]
        if self._arm_name == 'r':
            self._joint_neutral[0] = -self._joint_neutral[0]
            self._joint_neutral[2] = -self._joint_neutral[2]

        #publish zero velocities 
        self.set_joint_velocities([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        #initialize also joint position and velocity limits and default values
        self._rate = rospy.Rate(200) #the states are published at 500
        self._ee_pose = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
        self._ee_twist = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self._vel_KP = 0.1 #keep it low or the robot will block because it exceeds the force
        self._joint_threshold = 0.01

        #initialize the kinematic solvers
        self.initialize_kinematic_solvers()

        #set the stop on shutdown for safety
        rospy.on_shutdown(self.stop)

        #initialize the ros subsciber to keep track of the current joint state
        self._ros_topic_joint_states = rospy.get_param('/yumi/velocity_control/ros_topic_joint_states_name', '/yumi/joint_states')
        self._subscriber = rospy.Subscriber(self._ros_topic_joint_states, JointState, self.joint_state_callback)

    """this function sets the joint threshold"""
    def set_joint_postion_threshold(self, th):
        self._joint_threshold = th

    """this function returns the correct index in the list, which is different than the naming convention"""
    def redefine_index(self, idx):
        if idx < 1 or idx > 7:
            print idx
            rospy.logerr('Wrong joint index')
            #return a valid index anyway
            return 0
        elif idx < 3:
            # idx = 1 or idx =2
            return idx - 1
        elif idx == 7:
            return 2
        else:
            return idx

    """this function initializes a list of ros publishers for the joint velocities"""
    def initialize_ros_publishers(self):
        for idx in range(1,8):
            i = self.redefine_index(idx) #put the topic in the same order as the kinematic chain
            topic_name = self._ros_topic_namespace + str(idx) + '_' + self._arm_name + '/command'
            #print ('topic: ', i, ': ', topic_name)
            self._publishers[i] = rospy.Publisher(topic_name, Float64, queue_size = 1)

    """this function stops all the joints"""
    def stop(self):
        for i in range(0, 10):
            for p in self._publishers:
                p.publish(0.0)
                self._rate.sleep()

    """this function stops the desired joint"""
    def stop_joint(self, idx, redefine_idx = True):
        if redefine_idx:
            idx = self.redefine_index(idx)
        for i in range(0, 10):
            self._publishers[idx].publish(0.0)
            self._rate.sleep()

    """this function initializes all the kinematic solvers"""
    def initialize_kinematic_solvers(self):
        robot_description = rospy.get_param('/yumi/velocity_control/robot_description', '/robot_description')
        self._robot = URDF.from_parameter_server(key=robot_description)
        self._kdl_tree = kdl_tree_from_urdf_model(self._robot)
        self._base_link = self._robot.get_root()
        self._ee_link = 'gripper_' +self._arm_name + '_base' #name of the frame we want 
        self._ee_frame = PyKDL.Frame()
        self._ee_arm_chain = self._kdl_tree.getChain(self._base_link, self._ee_link)
        # KDL Solvers
        self._fk_p_kdl = PyKDL.ChainFkSolverPos_recursive(self._ee_arm_chain)
        self._fk_v_kdl = PyKDL.ChainFkSolverVel_recursive(self._ee_arm_chain)
        self._ik_v_kdl = PyKDL.ChainIkSolverVel_pinv(self._ee_arm_chain)
        #self._ik_p_kdl = PyKDL.ChainIkSolverPos_NR(self._arm_chain, self._fk_p_kdl, self._ik_v_kdl)
        self._ik_p_kdl = PyKDL.ChainIkSolverPos_LMA(self._ee_arm_chain)
        self._jac_kdl = PyKDL.ChainJntToJacSolver(self._ee_arm_chain)
        self._dyn_kdl = PyKDL.ChainDynParam(self._ee_arm_chain, PyKDL.Vector.Zero())

    """this function reads the joint_states message and updates the current state of the joints"""
    def joint_state_callback(self, data):
        names = data.name
        positions = data.position
        velocities = data.velocity
        for i, s in enumerate(names):
            if s[-1] == self._arm_name:
                idx = int (s[-3])
                index = self.redefine_index(idx) #put the joint positions according to the order in the kinematic chain
                self._joint_positions[index] = positions[i]
                self._joint_velocities[index] = velocities[i]
            #check if it is the gripper variable
            elif s[8] == self._arm_name:
                self._gripper_position = positions[i]
        #update the forward kinematics
        self.update_ee_state(self._joint_positions, self._joint_velocities)

    """this function moves joints to a desired position"""
    def set_joint_positions(self, positions, kinematic_chain_order = True, iterate = True, timeout = 20.0):
        #check the current overall error
        err = 0
        for i, p in enumerate(positions):
            idx = i
            if not kinematic_chain_order:
                idx = self.redefine_index(idx)
            err += abs(positions[i] - self._joint_positions[idx])
        if abs(err) < 7 * self._joint_threshold: 
            self.stop()
            return
        #iterate through all the joints
        t0 = rospy.Time.now().to_sec()
        t = rospy.Time.now().to_sec() - t0
        if iterate:
            while abs(err) > 7 * self._joint_threshold and t < timeout:
                err = 0
                for i, p in enumerate(positions):
                    idx = i
                    if not kinematic_chain_order:
                        idx = self.redefine_index(i + 1)
                    self.set_joint_position(idx, p, not kinematic_chain_order, False)
                    self._rate.sleep()
                    t = rospy.Time.now().to_sec() - t0
                    err += abs(positions[i] - self._joint_positions[idx])

            if t >= timeout:
                rospy.logwarn('Timeout exceeded before reaching target postion')
                self.stop()
            else:
                rospy.loginfo('Reached desired joint positions')
        else:
            for i, p in enumerate(positions):
                self.set_joint_position(i + 1, p, not kinematic_chain_order, False)

    """this function moves one joint to the desired position"""
    def set_joint_position(self, idx, position, redefine_idx = True, iterate = True, timeout = 20.0):
        if redefine_idx:
            i = self.redefine_index(idx)
        else:
            i = idx
            #here convert idx back (only for printouts)
        err = position - self._joint_positions[i]
        #check if the error is small enough (check what tolerance can be used for better precision)
        if abs(err) < self._joint_threshold: 
            self.stop_joint(i, False)
            return
        #use a p controller for now
        if iterate:
            t0 = rospy.Time.now().to_sec()
            t = rospy.Time.now().to_sec() - t0
            while abs(err) > self._joint_threshold and t < timeout:
                vel = self._vel_KP * err 
                self.set_joint_velocity(i, vel, False)
                self._rate.sleep()
                t = rospy.Time.now().to_sec() - t0
                err = position - self._joint_positions[i]

            if t >= timeout:
                rospy.logwarn('joint %d: Timeout exceeded before reaching target postion', idx)
                self.stop_joint(i, False)
            else:
                rospy.loginfo('joint %d: Reached desired position', idx)
        else:
            vel = self._vel_KP * err #check what gain to use
            self.set_joint_velocity(i, vel, False)

    """this function publishes all the joint velocities"""
    def set_joint_velocities(self, velocities, kinematic_chain_order = True):
        if kinematic_chain_order:
            for i, p in enumerate(self._publishers):
                p.publish(velocities[i])
        else:
            for i, p in enumerate(self._publishers):
                idx = self.redefine_index(i + 1)
                p.publish(velocities[i])


    """this function sets the desired velocity for the idx joint"""
    def set_joint_velocity(self, idx, vel, redefine_idx = True):
        if redefine_idx:
            idx = self.redefine_index(idx)
        self._publishers[idx].publish(vel)

    """this function returns the current joint positions"""
    def get_joint_positions(self):
        return self._joint_positions

    """this function returns the joint position for the specified joint"""
    def get_joint_position(self, idx, redefine_idx = True):
        if redefine_idx:
            idx = self.redefine_index(idx)
        return self._joint_positions[idx]

    """this function returns the current joint velocities"""
    def get_joint_velocities(self):
        return self._joint_velocities

    """this function return the joint velocity for the specified joint"""
    def get_joint_velocity(self, idx, redefine_idx = True):
        if redefine_index:
            idx = self.redefine_index(idx)
        return self._joint_velocities[idx]

    """this function returns joint values for the desired pose"""
    def get_inverse_kinematics(self, pose):
        #check pose of end effector
        pos = PyKDL.Vector(pose[0], pose[1], pose[2])
        rot = PyKDL.Rotation().Quaternion(pose[3], pose[4], pose[5], pose[6])
        seed_array = PyKDL.JntArray(len(self._joint_positions))

        for idx, val in enumerate(self._joint_positions):
            seed_array[idx] = val

        end_frame = PyKDL.Frame()
        self._fk_p_kdl.JntToCart(seed_array, end_frame)

        # Make IK Call
        goal_pose = PyKDL.Frame(rot, pos)
        result_angles = PyKDL.JntArray(len(self._joint_positions))

        if self._ik_p_kdl.CartToJnt(seed_array, goal_pose, result_angles) >= 0:
            rospy.loginfo("IK solution found!")
            return result_angles
        else:
            rospy.logwarn("No IK solution found.")
            return None

    """this function moves the arm to the desired pose"""
    def move_ee_to_pose(self, pose):
        joint_angles = self.get_inverse_kinematics(pose)
        if joint_angles is None:
            return
        else:
            self.set_joint_positions(joint_angles)

    """this function returns the joint velocities for the desired cartesian velocity of the end-effector(defined in the global reference frame)"""
    def get_inverse_kinematics_velocity(self, velocity):
        J = self.get_jacobian()
        vel = np.array(velocity)
        #invert J and compute the joint velocities
        q_dot = None
        if(len(velocity) == 3): #no twist (velocity has 3 element)
            J_sub = J[1:4, :] #take the submatrix corresponding to vx, vy, vz
            J_sub_pinv = np.linalg.pinv(J_sub)
            q_dot = J_sub_pinv.dot(vel)
        else:
            #assume the velocity has 6 elements 
            J_pinv = np.linalg.pinv(J)
            q_dot = J_pinv.dot(vel)

        return q_dot

    """this function returns the jacobian of the current arm configuration (as np array)"""
    def get_jacobian(self):
        #initialize seed array from current joint states
        seed_array = PyKDL.JntArray(len(self._joint_positions))
        for idx, val in enumerate(self._joint_positions):
            seed_array[idx] = val
        #compute the current jacobian and convert it into numpy array
        jacobian = PyKDL.Jacobian(len(self._joint_positions))
        self._jac_kdl.JntToJac(seed_array, jacobian)
        J = np.zeros([int(jacobian.rows()), int(jacobian.columns())])
        for i in range(int(jacobian.rows())):
            for j in range(int(jacobian.columns())):
                J[i, j] = jacobian[i, j]

        return J

    """this function sets the desired velocity for the end-effector (defined in the global reference frame)"""
    def set_ee_velocity(self, velocity):
        joint_vel = self.get_inverse_kinematics_velocity(velocity)
        if joint_vel is None:
            rospy.logerr('No feasible solution for joint velocities')
            return
        self.set_joint_velocities(joint_vel)

    """this function returns the direct kinematics of the end-effector (only pose for now)"""
    def get_forward_position_kinematics(self):
        return self._ee_pose

    def get_forward_velocity_kinematics(self):
        return self._ee_twist

    """this function updates the current forward kinematics"""
    def update_ee_state(self, joint_pos, joint_vel):
        end_frame = PyKDL.Frame()
        kdl_array = PyKDL.JntArray(len(joint_pos))
        for idx, val in enumerate(joint_pos):
            kdl_array[idx] = val
        self._fk_p_kdl.JntToCart(kdl_array, end_frame)
        self._ee_frame = end_frame
        pos = end_frame.p
        rot = PyKDL.Rotation(end_frame.M)
        rot = rot.GetQuaternion()
        self._ee_pose = np.array([pos[0], pos[1], pos[2], rot[0], rot[1], rot[2], rot[3]])

        end_frame = PyKDL.FrameVel()
        kdl_array = PyKDL.JntArray(len(joint_vel))
        for idx, val in enumerate(joint_vel):
            kdl_array[idx] = val
        kdl_array = PyKDL.JntArrayVel(kdl_array)
        self._fk_v_kdl.JntToCart(kdl_array, end_frame)
        self._ee_twist = end_frame.GetTwist() #not numpy array...

    """this function moves to a neutral joint position"""
    def move_to_neutral(self, timeout = 20.0):
        self.set_joint_positions(self._joint_neutral, timeout = timeout)

    """this function closes the gripper"""
    def close_gripper(self):
        try:
            self._close_service(self._gripper_id)
        except rospy.ServiceException, e:
            rospy.logerr("Close gripper service call failed: %s", %e)

    """this function opens the gripper"""
    def open_gripper(self):
        try:
            self._open_service(self._gripper_id)
        except rospy.ServiceException, e:
            rospy.logerr("Open gripper service call failed: %s", %e)

    """this function returns the distance of the gripper's fingers"""
    def get_finger_distance(self):
        return self._gripper_position




        