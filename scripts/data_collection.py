#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros
import numpy as np

from controller import Controller
from geometry_msgs.msg import PoseStamped
from dynamixel_workbench_msgs.srv import SetPID
from object_detection.srv import ObjectDetection

# Initialize controller
ctrl = Controller()
MIN_JOINT_MOTION_FOR_HAND_OVER = 0.1
# define state Idle
class Idle(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['gotToolInput'])

    def execute(self, userdata):
        ctrl.open_gripper()
        ctrl.set_camera_angles(ctrl.HOME_POS_CAMERA_01)
        ctrl.set_arm_joint_angles(ctrl.HOME_POS_MANIPULATOR_01)
        rospy.loginfo('Executing state IDLE')

        # Return success
        return 'gotToolInput'

class GrabTool(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['grabbedTool'])

    def execute(self, userdata):
        while(True):
            # TODO: Ask for Input
            ip = input("Enter a number (1-4) to start:")
            if(isinstance(ip, int) and ip>0 and ip<5):
                break
        # Return success
        return 'grabbedTool'

class Sample(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['gotSample'])

    def execute(self, userdata):
        
        # Return success
        return 'gotSample'

# define state IK1
class IK1(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['foundIK', 'noIK'])

    def execute(self, userdata):
        ## Wait till IK not found. Change tolerance and call compute IK again
        ## Break go to pose into find pose and compute IK functions
        ## Make a different function with gripper pointing upwards and call it from here
        success = ctrl.go_to_handover_pose(userdata.head_pose)
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
            for joint_num in userdata.joint_nums:
                response = set_PID(joint_num, P, I, D)
            return 'changed'
        except rospy.ServiceException as e:
            rospy.logwarn("Service call failed:{0}".format(e))
            return 'notchanged'

# define state OpenGripper
class OpenGripper(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['opened'])

    def execute(self, userdata):
        ## Detect when to Open gripper
        rospy.loginfo('Executing state OpenGripper')
        while(True):
            joint_len = len(ctrl.history['joint_feedback'][0])
            joint_sum = np.zeros(ctrl.history['joint_feedback'][0].shape)
            for joint_feedback in ctrl.history['joint_feedback']:
                joint_sum += joint_feedback
            avg_joint_sum = joint_sum/joint_len
            if(np.sum(avg_joint_sum) < MIN_JOINT_MOTION_FOR_HAND_OVER):
                ctrl.open_gripper()
                return 'opened'

# define state OpenGripper
class Evaluate(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done', 'exit'])

    def execute(self, userdata):
        # TODO: Ask for Input
        userdata.score = input("Enter score for this exchange (1 - 10):")
        if(isinstance(userdata.score, int) and userdata.score>0 and userdata.score<11):
            return 'done'
        else: 
            return 'exit'

def main():
    rospy.init_node('attention_bot')


    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['stop'])
    sm.userdata.tool_id = -1
    sm.userdata.joint_nums = [1,6]
    sm.userdata.PID = [0,0,0]
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
                               transitions={'changed':'OPENGRIPPER', 'notchanged': 'CHANGEPID'})
        smach.StateMachine.add('OPENGRIPPER', OpenGripper(),
                               transitions={'opened':'EVALUATE'})
        smach.StateMachine.add('EVALUATE', Evaluate(),
                               transitions={'done':'IDLE','exit':'stop'})

    # Execute SMACH plan
    outcome = sm.execute()
    sis.stop()

if __name__ == '__main__':
    main()