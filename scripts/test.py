#! /usr/bin/env python

"""
    This file contains a test for the python interface to the yumi robot with velocity control.

    @author: Silvia Cruciani (cruciani@kth.se)
"""

import rospy

from yumi_interface.yumi_arm_interface import YumiArm

if __name__ == '__main__':
    rospy.init_node('yumi_interface_test')

    left_arm = YumiArm('left')
    right_arm = YumiArm('right')

    #setting an end-effector pose:
    pose = [0.5, -0.2, 0.4, 0.0, 1.0, 0.0, 0.0]
    print("setting right end-effector to pose: ", pose)
    right_arm.move_ee_to_pose(pose)
    print ("reached: ", right_arm.get_forward_position_kinematics())
    rospy.sleep(2.0)
    print ("setting an effort of 18 on the gripper")
    right_arm.set_gripper_effort(18)
    rospy.sleep(2.0)
    print ("setting an effort of -10")
    right_arm.set_gripper_effort(-10)
    rospy.sleep(2.0)
    print("Setting horizontal linear velocity")
    r = rospy.Rate(100)
    for i in range(0, 100):
        right_arm.set_ee_velocity([0, -0.1, 0, 0, 0, 0])
        r.sleep()
    right_arm.stop()
    right_arm.stop()# do twice just in case
    print ("reached: ", right_arm.get_forward_position_kinematics())
