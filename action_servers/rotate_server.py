#! /usr/bin/env python

"""
    This file contains the rotate action for baxter

    @author: Silvia Cruciani (cruciani@kth.se)
"""

import sys
sys.path.insert(0, '../src/yumi_interface')
import rospy
import rospkg
import actionlib
import yumi_interface.msg

from yumi_arm_interface import YumiArm

import tf.transformations

import numpy as np

class RotateAction(object):
    def __init__(self, name):
        # create messages that are used to publish feedback/result
        self._feedback = yumi_interface.msg.RotateFeedback()
        self._result = yumi_interface.msg.RotateResult()

        #manipulation interface
        self._manipulation_interface = dict()
        self._manipulation_interface['right'] = YumiArm('right')
        self._manipulation_interface['left'] = YumiArm('left')

        #move both arms to the neutral position (assume no collision check is needed)
        #record a neutral arm position

        self._joint_threshold = rospy.get_param(name +'/joint_threshold', 0.05)
        

        #finally, start the action server
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, yumi_interface.msg.RotateAction, execute_cb = self.execute_cb, auto_start = False)
        self._as.start()
        self._r = rospy.Rate(100)

    """this function stops the motion in case of preemtion, and resets the previous position"""
    def preemption_reaction(self, arm):
        self._manipulation_interface.stop()
        rospy.loginfo('%s: Preempted' % self._action_name)
        self._manipulation_interface._baxter[arm]._limb.move_to_neutral(5.0)
        rospy.loginfo('Moved to neutral')

    def missing_target_frame_reaction(self):
        self._result.status = -2 #check which one is missing frame
        rospy.logerr('%s: Missing target frame' % self._action_name)
        if self._as.is_active():
            self._as.set_aborted(self._result)

    """this function resets the state in case of no IK solution"""
    def no_ik_solution_reaction(self, arm):
        self._result.status = -3 #check which one is no ik solution
        rospy.logerr('%s: No IK solution' % self._action_name)
        self._manipulation_interface._baxter[arm]._limb.move_to_neutral(5.0)
        rospy.loginfo('Moved to neutral')
        if self._as.is_active():
            self._as.set_aborted(self._result)
        return

    """this function stops the motion and resets the state in case of failure in reaching joint values"""
    def joints_not_reached_reaction(self, arm):
        self._result.status = -4 #check which one is fail in joint reaching
        rospy.loginfo('%s: Failed reaching reference pose' % self._action_name)
        self._manipulation_interface._baxter[arm]._limb.move_to_neutral(5.0)
        rospy.loginfo('Moved to neutral')
        if self._as.is_active():
            self._as.set_aborted(self._result)
        return

    """this function reaches the target joints for a robot arm"""
    def reach_joints(self, target_joints, arm):
        success = True
        #now move to the target joints
        cmd = self._manipulation_interface._baxter[arm]._limb.joint_angles() 
   
        def filtered_cmd(): 
            # First Order Filter - 0.2 Hz Cutoff 
            for joint in target_joints.keys(): 
                cmd[joint] = 0.012488 * target_joints[joint] + 0.98751 * cmd[joint] 
            return cmd 
 
        def joint_error():
            diff = 0
            for j, a in target_joints.items():
                diff = diff + abs(a - self._manipulation_interface._baxter[arm]._limb._joint_angle[j]) 
            return diff 

        joint_diff = joint_error()

        timeout = rospy.Duration(rospy.get_param(self._action_name + '/joint_motion_timeout', 15.0)) #use as parameter in launch
        init_t = rospy.get_rostime().to_sec()
        now = rospy.get_rostime().to_sec()
        execution_duration = rospy.Duration(now - init_t)

        while joint_diff > self._joint_threshold:
            #check for preemption
            if self._as.is_preempt_requested():
                self.preemption_reaction(arm)
                success = False
                return success

            #check if we exceeded the duration:
            now = rospy.get_rostime().to_sec()
            execution_duration = rospy.Duration(now - init_t)
            if execution_duration > timeout:
                self.joints_not_reached_reaction(arm)
                success = False
                return success

            self._feedback.status = 1 #define which one is the running feedback
            # publish the feedback
            self._as.publish_feedback(self._feedback)
            #keep setting the joints
            self._manipulation_interface._baxter[arm]._limb.set_joint_positions(filtered_cmd())
            self._r.sleep()
            joint_diff = joint_error()

        return success

    """this function approaches the object with the commanded velocity"""
    def approach(self, velocity, arm, final_approach_limit, limit_direction):
        success = True
        start_movement = False
        #store the initial joint position, to go back in case of preeption
        old_joint_positions = self._manipulation_interface._baxter[arm]._limb.joint_angles()

        #check if it is an approach or a retract motion
        reached = False
        retract = False
        endpoint_position = self._manipulation_interface._baxter[arm]._limb.endpoint_pose()['position']
        #print endpoint_position
        #print final_approach_limit

        if limit_direction =='x':
            retract = endpoint_position[0] < final_approach_limit
        elif limit_direction =='y':
            retract = endpoint_position[1] < final_approach_limit
        else:
            retract = endpoint_position[2] < final_approach_limit
        
        #print (retract)
        
        self._manipulation_interface.set_ee_velocity(velocity, arm)
        reached = False
        count = 0
        timeout = rospy.Duration(rospy.get_param(self._action_name + '/joint_motion_timeout', 15.0)) #use as parameter in launch
        init_t = rospy.get_rostime().to_sec()
        now = rospy.get_rostime().to_sec()
        execution_duration = rospy.Duration(now - init_t)
        while not reached:
            #check for preemption
            if self._as.is_preempt_requested():
                self._manipulation_interface._baxter[arm].stop()
                #go to old joint positions, then react to preemption            
                self._manipulation_interface._baxter[arm]._limb.move_to_joint_positions(old_joint_positions)
                self.preemption_reaction()
                self._as.set_preempted()
                success = False
                return success

            #check if we exceeded the duration:
            now = rospy.get_rostime().to_sec()
            execution_duration = rospy.Duration(now - init_t)
            if execution_duration > timeout:
                self.joints_not_reached_reaction(arm)
                success = False
                return success

            self._feedback.status = 1 #define which one is the running feedback
            # publish the feedback
            self._as.publish_feedback(self._feedback)

            self._manipulation_interface.set_ee_velocity(velocity, arm)

            if limit_direction == 'x':
                gripper_var = self._manipulation_interface._baxter[arm]._limb.endpoint_pose()['position'].x
                if abs(self._manipulation_interface._baxter[arm]._limb.endpoint_velocity()['linear'].x) > 0.05:
                    start_movement = True
                #check if the gripper is low enough now
                if((gripper_var <= final_approach_limit and not retract) or
                    (gripper_var >= final_approach_limit and retract)):
                    reached = True
                    self._manipulation_interface.set_ee_velocity(np.zeros(6), arm) #stop! we successfully reached the final approach pose

                #check if the velocity is too high (there is contact)
                if abs(self._manipulation_interface._baxter[arm]._limb.endpoint_velocity()['linear'].x) < 0.005 and start_movement:
                    #print ('reached velocity limit')
                    count = count + 1
                if count > 5:
                    self._manipulation_interface.set_ee_velocity(np.zeros(6), arm) #stop! we reached a final approach pose (could be not the desired one)
                    reached = True


            elif limit_direction == 'y':
                gripper_var = self._manipulation_interface._baxter[arm]._limb.endpoint_pose()['position'].y
                if abs(self._manipulation_interface._baxter[arm]._limb.endpoint_velocity()['linear'].y) > 0.05:
                    start_movement = True
                #check if the gripper is low enough now
                if((gripper_var <= final_approach_limit and not retract) or
                    (gripper_var >= final_approach_limit and retract)):
                    reached = True
                    self._manipulation_interface.set_ee_velocity(np.zeros(6), arm) #stop! we successfully reached the final approach pose

                #check if the velocity is too high (there is contact)
                if abs(self._manipulation_interface._baxter[arm]._limb.endpoint_velocity()['linear'].y) < 0.005 and start_movement:
                    #print ('reached velocity limit')
                    count = count + 1
                if count > 5:
                    self._manipulation_interface.set_ee_velocity(np.zeros(6), arm) #stop! we reached a final approach pose (could be not the desired one)
                    reached = True
            

            else:
                gripper_var = self._manipulation_interface._baxter[arm]._limb.endpoint_pose()['position'].z
                if abs(self._manipulation_interface._baxter[arm]._limb.endpoint_velocity()['linear'].z) > 0.05:
                    start_movement = True
                #check if the gripper is low enough now
                if((gripper_var <= final_approach_limit and not retract) or
                    (gripper_var >= final_approach_limit and retract)):
                    reached = True
                    self._manipulation_interface.set_ee_velocity(np.zeros(6), arm) #stop! we successfully reached the final approach pose

                #check if the velocity is too high (there is contact)
                if abs(self._manipulation_interface._baxter[arm]._limb.endpoint_velocity()['linear'].z) < 0.005 and start_movement:
                    #print ('reached velocity limit')
                    count = count + 1
                if count > 5:
                    self._manipulation_interface.set_ee_velocity(np.zeros(6), arm) #stop! we reached a final approach pose (could be not the desired one)
                    reached = True

            self._r.sleep()

        return success
        
    """this is where everything happens"""
    def execute_cb(self, goal):
        # helper variables
        success = True

        # publish info to the console for the user
        rospy.loginfo('%s: Received the goal %s', self._action_name, goal.target)

        #get the desired target frame and designated arm
        target = goal.target
        arm = goal.arm
        rotation = goal.rotation
        if not (arm =='right' or arm == 'left'):
            rospy.logerr('%s :Invalid arm name', self._action_name)
            if self._as.is_active():
                self._as.set_aborted(self._result)
            return

        #store the necessary tfs
        target_position, target_orientation = self._manipulation_interface.get_pose(target)

        #check if the taget frame exists
        if target_position is None:
            self.missing_target_frame_reaction()
            return

        #define the first in between frame (equal to target but higher?)

        trans_mat = tf.transformations.translation_matrix(target_position)
        rot_mat   = tf.transformations.quaternion_matrix(target_orientation)
        mat2 = np.dot(trans_mat, rot_mat) #from target frame to world
        approach_z = target_position[2] -0.03

        
        #transform to get the approach pose:
        trans_mat = tf.transformations.translation_matrix([0.0, 0.0, 0.01]) #10cm displacement on z
        rot_mat   = tf.transformations.quaternion_matrix([1.0, 0.0, 0.0, 0.0]) #180 deg rotation around x
        mat1 = np.dot(trans_mat, rot_mat) #from approach 2 to target frame

        mat1 = np.dot(mat2, mat1) #from approach to world
        retract_z = mat1[2][3]

        #get the target joints for the approach pose:
        position = tf.transformations.translation_from_matrix(mat1)
        orientation_q = tf.transformations.quaternion_from_matrix(mat1)
        orientation = tf.transformations.euler_from_quaternion(orientation_q)
        target_joints = self._manipulation_interface._baxter[arm].get_joint_values([position[0], position[1], position[2], orientation[0], orientation[1], orientation[2]])
        if target_joints is None:
            self.no_ik_solution_reaction(arm)
            return

        #send the target joints
        success = self.reach_joints(target_joints, arm)

        if not success:
            #the action settings have already been solved. No need to continue
            return

        #now open the gripper
        self._manipulation_interface._baxter[arm]._gripper.open(True, 3.0)

        

        #and now approach        
        velocity = np.array([0, 0, -0.1, 0, 0, 0]) #no twist, move down vertically
        
        #send velocity
        success = self.approach(velocity, arm, approach_z, 'z') #the approach motion is along the z direction in this case
        if not success:
            return

        #now that we finally reached our destination, we can close and rotate
        self._manipulation_interface._baxter[arm]._gripper.close(True, 3.0)

        #get the current wrist joint value
        joint_name = arm + '_w2'
        init_angle = self._manipulation_interface._baxter[arm]._limb.joint_angle(joint_name)
        #print init_angle
        desired_angle = init_angle + rotation
        #print desired_angle

        joint_position = dict()
        joint_position[joint_name] = desired_angle

        joint_reached = False
        stuck = False
        count = 0

        joint_value = self._manipulation_interface._baxter[arm]._limb.joint_angle(joint_name)

        while not joint_reached and not stuck:
            init_angle = self._manipulation_interface._baxter[arm]._limb.set_joint_positions(joint_position)
            prev_joint = joint_value
            joint_value = self._manipulation_interface._baxter[arm]._limb.joint_angle(joint_name)
            #check if the joint value is close to the desired one
            if abs(joint_value - desired_angle) < 0.05:
                joint_reached
            #check if the joint value is not changing
            if abs(joint_value - prev_joint) < 0.05:
                count +=1
            else:
                count = 0

            #if it has not moved for a while, stop trying. Assume there is nothing we can do
            if count > 50:
                stuck = True

            self._r.sleep()

        #now open
        self._manipulation_interface._baxter[arm]._gripper.open(True, 3.0)

        #now go back
        velocity = - velocity #in the opposite direction
        success = self.approach(velocity, arm, retract_z, 'z')
        if not success:
            return 

        #send to neutral
        self._manipulation_interface._baxter[arm]._limb.move_to_neutral()       

        self._result.status = 1 #check which one is success
        rospy.loginfo('%s: Succeeded' % self._action_name)
        self._as.set_succeeded()


if __name__ == '__main__':
    rospy.init_node('rotate_server')
    server = RotateAction('/baxter/rotate')
    #for safe interrupt handling
    rospy.on_shutdown(server._manipulation_interface.stop)
    rospy.spin()
