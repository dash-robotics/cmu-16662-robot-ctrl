#!/usr/bin/env python

import os
import roslib
import rospy
import smach
import smach_ros
import numpy as np
import tf

from controller import Controller
from geometry_msgs.msg import PoseStamped
from dynamixel_workbench_msgs.srv import SetPID
from object_detection.srv import ObjectDetection

from geometry_msgs.msg import Pose, Point, Quaternion

from gp import GaussianProcess

# Initialize controller
ctrl = Controller()
# Initialize GP
gp = GaussianProcess()
gp.fit_GP()
MIN_JOINT_MOTION_FOR_HAND_OVER = 0.1
# define state Idle
class Idle(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['gotToolInput'])

    def execute(self, userdata):
        ctrl.open_gripper()
        # ctrl.set_camera_angles(ctrl.HOME_POS_CAMERA_01)
        ctrl.set_arm_joint_angles(ctrl.HOME_POS_MANIPULATOR_00)
        rospy.loginfo('Executing state IDLE')

        # Return success
        return 'gotToolInput'

class GrabTool(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['grabbedTool'])

    def execute(self, userdata):
        while(True):
            # TODO: Ask for Input
            ip = input("Enter 1 to grab tool:")
            if(isinstance(ip, int) and ip==1):
                ctrl.close_gripper()
                break
        return 'grabbedTool'

class Sample(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['gotSample'], input_keys=['sample_constraint_mean','sample_constraint_cov', 'sampled', 'PID_Final'], output_keys=['sampled', 'PID_Final'])

    def execute(self, userdata):
        for idx, (constraint_mean, constraint_cov) in enumerate(zip(userdata.sample_constraint_mean, userdata.sample_constraint_cov)):
            #userdata.sampled[idx] = np.random.uniform(constraint_min, constraint_max)
            userdata.sampled[idx] = np.random.normal(constraint_mean, constraint_cov)
        userdata.PID_Final[0:3] = userdata.sampled[4:7]
        gp.find_reward(userdata.sampled)
        # Return success
        return 'gotSample'

# define state IK1
class IK1(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['foundIK', 'noIK'], input_keys=['point','sampled'])

    def execute(self, userdata):
        ## Wait till IK not found. Change tolerance and call compute IK again
        ## Break go to pose into find pose and compute IK functions
        ## Make a different function with gripper pointing upwards and call it from here

        Q = tf.transformations.quaternion_from_euler(userdata.sampled[0], userdata.sampled[1], userdata.sampled[2])
        
        pose_input = Pose(Point(userdata.point[0], userdata.point[1], userdata.point[2]), Quaternion(Q[0], Q[1], Q[2], Q[3]))
        
        success = ctrl.go_to_data_collection_pose(pose_input)
        # if not success
        rospy.loginfo('Executing state IK1')
        return 'foundIK'

class ChangePID(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['changed', 'notchanged'], input_keys=['joint_nums','PID'])

    def execute(self, userdata):
        rospy.wait_for_service('SetPID')
        try:
            set_PID = rospy.ServiceProxy('SetPID', SetPID)
            P,I,D = userdata.PID
            if P < 0:
                P = 0
            if I < 0:
                I  = 0
            if D < 0:
                D = 0
            rospy.loginfo('PID Values are as follows P:' + str(P) + ' I:' + str(I) + ' D:' + str(D))
            for joint_num in userdata.joint_nums:
                response = set_PID(joint_num, P, I, D)
            return 'changed'
        except rospy.ServiceException as e:
            rospy.logwarn("Service call failed:{0}".format(e))
            return 'notchanged'

# define state OpenGripper
class OpenGripper(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['opened'], input_keys=['sampled'])

    def execute(self, userdata):
        ## Detect when to Open gripper
        rospy.loginfo('Executing state OpenGripper')
        rospy.logwarn('Threshold value is :' + str(userdata.sampled[3]))
        rospy.sleep(2)
        while(True):
            avg_joint_sum = np.sum(np.abs(np.asarray(ctrl.current_target_state) - np.asarray(ctrl.current_joint_state)[ctrl.MOVEABLE_JOINTS]))
            # joint_len = len(ctrl.history['joint_feedback'][0])
            # joint_sum = np.zeros(ctrl.history['joint_feedback'][0].shape)
            # for joint_feedback in list(ctrl.history['joint_feedback']):
            #     #pdb.set_trace()
            #     joint_sum += joint_feedback - ctrl.current_target_state
            # avg_joint_sum = joint_sum/joint_len
            # print(joint_sum)
            # rospy.loginfo("Total change in joint angles: " + str(np.sum(np.abs(avg_joint_sum))))
            if(np.sum(np.abs(avg_joint_sum)) >  userdata.sampled[3]):
                ctrl.open_gripper()
                return 'opened'

# define state OpenGripper
class Evaluate(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done', 'idle', 'exit'], input_keys=['sampled'])

    def execute(self, userdata):
        # TODO: Ask for Input
        save_location = '../data/dataset_collection.npy'
        score = input("Enter score for this exchange (1 - 10):")
        if(isinstance(score, int) and score>0 and score<11):
            y = np.load(save_location) if os.path.isfile(save_location) else []
            x = np.append(userdata.sampled, score)
            if y == []:
                np.save(save_location,x)
            else:
                np.save(save_location,np.vstack((y,x)))
            return 'done'
        elif(isinstance(score, int) and score == 0): 
            return 'idle'
        else:
            return 'exit'

def main():
    rospy.init_node('attention_bot')


    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['stop'])
    sm.userdata.tool_id = -1
    sm.userdata.joint_nums = [0,1,2,5]
    # sm.userdata.sample_constraint_min = [-np.pi/3,-np.pi/3,-np.pi/3, 0.02, 100, 0, 0]
    # sm.userdata.sample_constraint_max = [np.pi/3,np.pi/3,np.pi/3, 0.15, 650, 10, 100]
    sm.userdata.sample_constraint_mean = [0, 0, 0, 0.05, 150, 2, 20]
    sm.userdata.sample_constraint_cov = [np.pi/9, np.pi/9, np.pi/9, 0.01, 50, 0.6, 5]
    sm.userdata.sampled = [0, 0, 0, 0, 0, 0, 0]
    sm.userdata.point = [0.4, 0, 0.20]
    sm.userdata.PID_Final = [250,0,0]
    sm.userdata.PID_Initial = [800,5,50]
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()
    rospy.sleep(5)
    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('IDLE', Idle(),
                               transitions={'gotToolInput':'GRABTOOL'})
        smach.StateMachine.add('GRABTOOL', GrabTool(),
                               transitions={'grabbedTool':'SAMPLE'})
        smach.StateMachine.add('SAMPLE', Sample(),
                                transitions={'gotSample':'IK1'})
        smach.StateMachine.add('IK1', IK1(),
                               transitions={'noIK':'SAMPLE','foundIK':'CHANGEPID'})
        smach.StateMachine.add('CHANGEPID', ChangePID(),
                               transitions={'changed':'OPENGRIPPER', 'notchanged': 'CHANGEPID'},
                               remapping={'PID':'PID_Final'})
        smach.StateMachine.add('OPENGRIPPER', OpenGripper(),
                               transitions={'opened':'CHANGEBACKPID'})
        smach.StateMachine.add('CHANGEBACKPID', ChangePID(),
                               transitions={'changed':'EVALUATE', 'notchanged': 'CHANGEPID'},
                               remapping={'PID':'PID_Initial'})
        smach.StateMachine.add('EVALUATE', Evaluate(),
                               transitions={'done':'IDLE', 'idle':'IDLE', 'exit':'stop'})

    # Execute SMACH plan
    outcome = sm.execute()
    sis.stop()

if __name__ == '__main__':
    main()