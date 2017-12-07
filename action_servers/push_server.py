#! /usr/bin/env python

"""
    This file contains the push action for yumi

    @author: Silvia Cruciani (cruciani@kth.se)
"""

import sys
import rospy
import rospkg

sys.path.insert(0, rospkg.RosPack.get_path(rospkg.RosPack(), 'yumi_interface') + '/src/yumi_interface')
import actionlib
import yumi_interface.msg

from yumi_arm_interface import YumiArm

import tf.transformations
import tf

import numpy as np

class PushAction(object):
    def __init__(self, name):
        # create messages that are used to publish feedback/result
        self._feedback = yumi_interface.msg.PushFeedback()
        self._result = yumi_interface.msg.PushResult()

        #for debug purposes
        self._broadcaster = tf.TransformBroadcaster()
        #tf listener
        self._tf_listener = tf.TransformListener()

        #manipulation interface
        self._manipulation_interface = dict()
        self._manipulation_interface['right'] = YumiArm('right')
        self._manipulation_interface['left'] = YumiArm('left')

        self._base_frame = rospy.get_param(name + '/base_frame', 'world')

        self._joint_threshold = rospy.get_param(name +'/joint_threshold', 0.05)
        self._joint_movement_threshold = rospy.get_param(name +'/joint_movement_threshold', 0.05)
        self._joint_stop_threshold = rospy.get_param(name +'/joint_stop_threshold', 0.005)

        #set the threshold for the joint position/velocity control
        self._manipulation_interface['right'].set_joint_position_threshold(0.01)
        self._manipulation_interface['left'].set_joint_position_threshold(0.01)

        #move both arms to the neutral position (assume no collision check is needed)
        #find a neutral position
        self._right_neutral = [-0.23, -1.82, -1.0, 0.0, 2.0, -1.4, 0.0]
        self._left_neutral = [0.3, -2.0, 0.8, 0.0, 1.0, 1.6, -0.5]

        self._manipulation_interface['right']._joint_neutral = self._right_neutral
        self._manipulation_interface['left']._joint_neutral = self._left_neutral

        #ovverride default neutral position
        self._manipulation_interface['right'].move_to_neutral(10.0)
        self._manipulation_interface['left'].move_to_neutral(10.0)

        #finally, start the action server
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, yumi_interface.msg.PushAction, execute_cb = self.execute_cb, auto_start = False)
        self._as.start()
        self._r = rospy.Rate(100)

    """this function stops all the motion for safety purposes"""
    def stop_motion(self):
        self._manipulation_interface['right'].stop()
        self._manipulation_interface['left'].stop()

    """this function reads the target pose from tf"""
    def get_pose(self, obj_name):
        if self._tf_listener.frameExists(obj_name):
            t = rospy.Time(0)
            try:
                position, quaternion = self._tf_listener.lookupTransform(self._base_frame, obj_name, rospy.Time(0))
                return position, quaternion
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                rospy.logerr('tf error')
                return None, None
        else:
            rospy.logerr("No Transform between: " + self._base_frame + " and " + obj_name)
            return None, None

    """this function stops the motion in case of preemtion, and resets the previous position"""
    def preemption_reaction(self, arm):
        self._manipulation_interface[arm].stop()
        rospy.loginfo('%s: Preempted' % self._action_name)
        self._manipulation_interface[arm].move_to_neutral(10.0)
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
        self._manipulation_interface[arm].move_to_neutral(10.0)
        rospy.loginfo('Moved to neutral')
        if self._as.is_active():
            self._as.set_aborted(self._result)
        return

    """this function stops the motion and resets the state in case of failure in reaching joint values"""
    def joints_not_reached_reaction(self, arm):
        self._result.status = -4 #check which one is fail in joint reaching
        rospy.loginfo('%s: Failed reaching reference pose' % self._action_name)
        self._manipulation_interface[arm].move_to_neutral(10.0)
        rospy.loginfo('Moved to neutral')
        if self._as.is_active():
            self._as.set_aborted(self._result)
        return

    """this function reaches the target joints for a robot arm"""
    def reach_joints(self, target_joints, arm):
        success = True
        #now move to the target joints
        cmd = self._manipulation_interface[arm].get_joint_positions()
   
        def filtered_cmd(): 
            # First Order Filter - 0.2 Hz Cutoff 
            for i, target in enumerate(target_joints): 
                cmd[i] = 0.012488 * target + 0.98751 * cmd[i] 
            return cmd 
 
        def joint_error():
            diff = 0
            for j, a in enumerate(target_joints):
                diff = diff + abs(a - self._manipulation_interface[arm].get_joint_position(j, False)) 
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
            self._manipulation_interface[arm].set_joint_positions(filtered_cmd(), iterate = False)
            self._r.sleep()
            joint_diff = joint_error()

        return success

    """this function does the approach/retract motion"""
    def command_approach_motion(self, i, arm, start_movement, reached, count, retract, final_approach_limit):
        gripper_var = self._manipulation_interface[arm].get_forward_position_kinematics()[i]
        if abs(self._manipulation_interface[arm].get_forward_velocity_kinematics()[i]) > self._joint_movement_threshold:
            start_movement = True
        #check if the gripper is low enough now
        if((gripper_var <= final_approach_limit and not retract) or
            (gripper_var >= final_approach_limit and retract)):
            reached = True
            self._manipulation_interface[arm].stop() #stop! we successfully reached the final approach pose

        #check if the velocity is too high (there is contact)
        if abs(self._manipulation_interface[arm].get_forward_velocity_kinematics()[i]) < self._joint_stop_threshold and start_movement:
            #print ('reached velocity limit')
            count = count + 1
        if count > 5:
            self._manipulation_interface[arm].stop() #stop! we reached a final approach pose (could be not the desired one)
            reached = True
        return start_movement, reached, count

    """this function approaches the object with the commanded velocity"""
    def approach(self, velocity, arm, final_approach_limit, limit_direction):
        success = True
        start_movement = False
        #store the initial joint position, to go back in case of preeption
        old_joint_positions = self._manipulation_interface[arm].get_joint_positions()

        #check if it is an approach or a retract motion
        reached = False
        retract = False
        endpoint_position = self._manipulation_interface[arm].get_forward_position_kinematics()[0:3]

        if limit_direction =='x':
            retract = endpoint_position[0] < final_approach_limit
        elif limit_direction =='y':
            retract = endpoint_position[1] < final_approach_limit
        else:
            retract = endpoint_position[2] < final_approach_limit
        
        self._manipulation_interface[arm].set_ee_velocity(velocity)
        reached = False
        count = 0
        timeout = rospy.Duration(rospy.get_param(self._action_name + '/joint_motion_timeout', 15.0)) #use as parameter in launch
        init_t = rospy.get_rostime().to_sec()
        now = rospy.get_rostime().to_sec()
        execution_duration = rospy.Duration(now - init_t)
        while not reached:
            #check for preemption
            if self._as.is_preempt_requested():
                self._manipulation_interface[arm].stop()
                #go to old joint positions, then react to preemption            
                self._manipulation_interface[arm].set_joint_positions(old_joint_positions)
                self.preemption_reaction(arm)
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

            self._manipulation_interface[arm].set_ee_velocity(velocity)

            if limit_direction == 'x':
                start_movement, reached, count = self.command_approach_motion(0, arm, start_movement, reached, count, retract, final_approach_limit)
            elif limit_direction == 'y':
                start_movement, reached, count = self.command_approach_motion(1, arm, start_movement, reached, count, retract, final_approach_limit)
            else:
                start_movement, reached, count = self.command_approach_motion(2, arm, start_movement, reached, count, retract, final_approach_limit)

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
        push_direction = goal.direction
        approach_distance = goal.distance
        if not (arm =='right' or arm == 'left'):
            rospy.logerr('%s :Invalid arm name', self._action_name)
            if self._as.is_active():
                self._as.set_aborted(self._result)
            return
        push_direction = np.array(push_direction)
        if np.linalg.norm(push_direction) < 0.000001:
            rospy.logerr('%s :Invalid push direction', self._action_name)
            if self._as.is_active():
                self._as.set_aborted(self._result)
            return

        #store the necessary tfs
        target_position, target_orientation = self.get_pose(target)
        retract_z = self._manipulation_interface[arm].get_forward_position_kinematics()[2]

        #check if the taget frame exists
        if target_position is None:
            self.missing_target_frame_reaction()
            return

        #define the first in between frame (equal to target but higher?)

        trans_mat = tf.transformations.translation_matrix(target_position)
        rot_mat   = tf.transformations.quaternion_matrix(target_orientation)
        mat3 = np.dot(trans_mat, rot_mat) #from target frame to world

        #transform the approach direction into the target frame
        mat3_inverse = np.linalg.inv(mat3) #from world to target
        normalized_direction = push_direction/np.linalg.norm(push_direction)
        transformed_direction = np.dot(mat3_inverse, - approach_distance * np.append(normalized_direction, 0)) 
        
        #transform to get the second approach pose:
        #trans_mat = tf.transformations.translation_matrix([0.0, 0.1, 0.1]) #10cm displacement on z and 10 on y (gripper base, not fingertips)
        trans_mat = tf.transformations.translation_matrix([transformed_direction[0], transformed_direction[1], 0.15])
        beta = np.arctan2(transformed_direction[0], -transformed_direction[1])
        rot_mat1 = tf.transformations.euler_matrix(0, 0, beta)
        rot_mat2 = tf.transformations.quaternion_matrix([0.70710678, 0.70710678, 0.0,  0.0]) #90 deg rotation on 
        #rot_mat2 = tf.transformations.quaternion_matrix([0.70710678, 0.0,  0.0,  0.70710678]) #90 deg rotation on 
        rot_mat = np.dot(rot_mat1, rot_mat2)
        mat2 = np.dot(trans_mat, rot_mat) #from approach 2 to target frame

        mat2 = np.dot(mat3, mat2) #from approach 2 to world

        #transform to get the first approach pose: (somehow make these all parameters, or configurable)
        trans_mat = tf.transformations.translation_matrix([0.0, 0.0, -0.2]) #-20cm displacement on z
        rot_mat   = tf.transformations.quaternion_matrix([0.0, 0.0,  0.0,  1.0]) #no rotation
        mat1 = np.dot(trans_mat, rot_mat) #from approach 1 to approach 2 frame

        mat1 = np.dot(mat2, mat1) #from approach 1 to world

        #first of all, close the gripper
        self._manipulation_interface[arm].close_gripper()

        #get the target joints for the first approach pose:
        position = tf.transformations.translation_from_matrix(mat1)
        orientation_quat = tf.transformations.quaternion_from_matrix(mat1)

        #store the first approach z for retraction
        retract_z = position[2]


        self._broadcaster.sendTransform(position, orientation_quat, rospy.Time.now(), "approach_1", "world")
        target_joints = self._manipulation_interface[arm].get_inverse_kinematics([position[0], position[1], position[2], orientation_quat[0], orientation_quat[1], orientation_quat[2], orientation_quat[3]])
        if target_joints is None:
            self.no_ik_solution_reaction(arm)
            return

        rospy.loginfo('approaching the target')

        #send the target joints
        #success = self.reach_joints(target_joints, arm)
        self._manipulation_interface[arm].set_joint_positions(target_joints)

        if not success:
            #the action settings have already been solved. No need to continue
            return

        #the first part of the execution succeeded. Now we need to approach and then push
        
        velocity = np.array([0, 0, -0.1, 0, 0, 0]) #no twist, move vertically

        self._broadcaster.sendTransform(tf.transformations.translation_from_matrix(mat2), tf.transformations.quaternion_from_matrix(mat2), rospy.Time.now(), "approach_2", "world")

        rospy.loginfo('approaching the push start point')
        
        #send velocity
        success = self.approach(velocity, arm, tf.transformations.translation_from_matrix(mat2)[2], 'z') #the approach motion is along the z direction in this case
        if not success:
            return

        #now that we finally reached our destination, we can do the push action
        vel = rospy.get_param(self._action_name + '/velocity_magnitude', 0.1)
        velocity = vel*np.append(np.array(normalized_direction), np.array([0, 0, 0])) #impose no twist
        rospy.loginfo("sending velocity: %s", str(velocity))

        rospy.loginfo('begin the push action')

        success = self.approach(velocity, arm, target_position[0] + 0.0, 'x') #assume this is it for now (along x mostly)

        if not success:
            return

        #now go back
        rospy.loginfo('retracting')

        velocity = - velocity
        success = self.approach(velocity, arm, target_position[0] - 0.0, 'x') #assume this is it for now (along x mostly)
        if not success:
            return

        velocity = np.array([0, 0, 0.1, 0, 0, 0]) #no twist, move vertically
        #check why it has to be recomputed
        #retract_z = self._manipulation_interface[arm].get_forward_position_kinematics()[2]
        success = self.approach(velocity, arm, retract_z + 0.0, 'z') #go up 5 cm
        if not success:
            return
        
        #send to neutral
        self._manipulation_interface[arm].move_to_neutral()
          
        self._result.status = 1 #check which one is success
        rospy.loginfo('%s: Succeeded' % self._action_name)
        self._as.set_succeeded()


if __name__ == '__main__':
    rospy.init_node('push_server')
    server = PushAction('/yumi/push')
    #for safe interrupt handling
    rospy.on_shutdown(server.stop_motion)
    rospy.spin()
