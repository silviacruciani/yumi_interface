#! /usr/bin/env python

"""
    Defines a simple move server for yumi

    @author: Silvia Cruciani (cruciani@kth.se)
    @author: Diogo Almeida (diogoa@kth.se)
"""

import sys
import rospy
import rospkg

sys.path.insert(0, rospkg.RosPack.get_path(rospkg.RosPack(), 'yumi_interface') + '/src/yumi_interface')
import rospy
import rospkg
import actionlib
import yumi_interface.msg

from yumi_arm_interface import YumiArm

import tf.transformations

import numpy as np

arms = ['right', 'left']

class MoveAction(object):
    def __init__(self, name):
        # create messages that are used to publish feedback/result
        self._feedback = yumi_interface.msg.MoveFeedback()
        self._result = yumi_interface.msg.MoveResult()

        self._tf_listener = tf.TransformListener()
        self._base_frame = rospy.get_param('~base_frame', False)

        if not self._base_frame:
            rospy.logerr("%s: base_frame not defined!", name)
            raise Exception('Missing base parameter from MoveAction class')

        #manipulation interface
        self._manipulation_interface = dict()
        self._joint_threshold = rospy.get_param('~joint_threshold', 0.05)
        self._joint_movement_threshold = rospy.get_param('~joint_movement_threshold', 0.05)
        self._joint_stop_threshold = rospy.get_param('~joint_stop_threshold', 0.005)

        for arm in arms:
            self._manipulation_interface[arm] = YumiArm(arm)
            self._manipulation_interface[arm].set_joint_position_threshold(self._joint_threshold)

        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, yumi_interface.msg.MoveAction, execute_cb = self.execute_cb, auto_start = False)
        self._as.start()
        rospy.loginfo("Started the move action server for Yumi!")
        self._r = rospy.Rate(100)

    def execute_cb(self, goal):
        rospy.logdebug('%s: received a goal!', self._action_name)

        arm = goal.arm

        if not (arm == goal.RIGHT_ARM or arm == goal.LEFT_ARM):
            rospy.logerr('%s: Invalid arm name', self._action_name)

            if self._as.is_active():
                self._as.set_aborted(self._result)
            return

        if goal.use_pose_target:
            target_pose = goal.cartesian_target
            target_pose = self._tf_listener.transformPose(self._base_frame, target_pose)  # convert received pose to the kinematic chain base
            p = target_pose.position
            o = target_pose.orientation
            target_joints = self._manipulation_interface[arm].get_inverse_kinematics([p[0], p[1], p[2], o[0], o[1], o[2], o[3]])
            if target_joints is None:
                self._result.status = self._result.NO_IK_SOLUTION
                if self._as.is_active():
                    self._as.set_aborted(self._result)
                return
        else:
            if len(goal.joint_target) != 7:
                rospy.logerr("Got joint target of invalid length: %d. Must be 7", len(goal.joint_target))
                self._result.status = self._result.INVALID_GOAL
                if self._as.is_active():
                    self._as.set_aborted(self._result)
                return

            target_joints = goal.joint_target

        success = self._manipulation_interface[arm].set_joint_positions(target_joints)  # TODO: make preemptable
        self._manipulation_interface[arm].set_joint_velocities([0., 0., 0., 0., 0., 0., 0.])  # TODO: make preemptable

        if not success:
            self._result.status = self._result.FAIL_TO_REACH_JOINT_TARGET
            if self._as.is_active():
                self._as.set_aborted(self._result)
            return

        self._as.set_succeeded()

if __name__ == '__main__':
    rospy.init_node('move_server')
    server = MoveAction('/yumi/move')
    rospy.spin()
