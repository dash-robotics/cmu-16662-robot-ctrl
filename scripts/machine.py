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
        smach.State.__init__(self, outcomes=['gotToolInput'], input_keys=['tool_id'], output_keys=['tool_id'])

    def execute(self, userdata):
        userdata.tool_id = -1
        ctrl.open_gripper()
        ctrl.set_camera_angles(ctrl.HOME_POS_CAMERA_01)
        ctrl.set_arm_joint_angles(ctrl.HOME_POS_MANIPULATOR_01)
        rospy.loginfo('Executing state IDLE')
        while(True):
            # TODO: Ask for Input
            userdata.tool_id = input("Enter Tool ID you want:")
            if(isinstance(userdata.tool_id, int) and userdata.tool_id>0 and userdata.tool_id<5):
                break

        # Return success
        return 'gotToolInput'



# define state FindTool
class FindTool(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['foundTool', 'notfoundTool'], input_keys=['tool_id'], output_keys=['frame_name'])

    def execute(self, userdata):
        rospy.loginfo('Executing state FINDTOOL')
        rospy.wait_for_service('detect_object')
        try:
            get_success = rospy.ServiceProxy('detect_object', ObjectDetection)
            success = get_success(tool_id)
            if success:
                userdata.frame_name = "/icp_cuboid_frame"
                return 'foundTool'
            else:
                rospy.logwarn("Service returned false")
                return 'notfoundTool'
        except: rospy.ServiceException as e:
            rospy.logwarn("Service called failed:{0}".format(e))
            return 'notfoundTool'

# define state IK1
class IK1(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['noIK','foundIK'], input_keys=['frame_name'])

    def execute(self, userdata):
        ctrl.set_arm_joint_angles(ctrl.HOME_POS_MANIPULATOR_00)
        rospy.loginfo('Executing state IK1')
        success = ctrl.go_to_grasp_pose(userdata.frame_name)
        return 'foundIK' if success else 'noIK'
        # return 'foundIK'

# define state Move
# class Move(smach.State):
#     def __init__(self):
#         smach.State.__init__(self, outcomes=['reached'])

#     def execute(self, userdata):
#         rospy.loginfo('Executing state IK1')
#         return 'reached'

# define state Grab
class Grab(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['grabSuccess', 'grabFailure'])

    def execute(self, userdata):
        ctrl.close_gripper()
        rospy.loginfo('Executing state IK1')
        if ctrl.check_grasp():
            return 'grabSuccess'
        else:
            return 'grabFailure'

# define state MoveHome2
class MoveHome2(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['reached'])

    def execute(self, userdata):
        ctrl.set_camera_angles(ctrl.HOME_POS_CAMERA_02)
        ctrl.set_arm_joint_angles(ctrl.HOME_POS_MANIPULATOR_02)
        rospy.loginfo('Executing state MoveHome2')
        return 'reached'

# define state OreintCamera
# class OrientCamera(smach.State):
#     def __init__(self):
#         smach.State.__init__(self, outcomes=['reached'])

#     def execute(self, userdata):
#         rospy.loginfo('Executing state OrientCamera')
#         return 'reached'

# define state FindAttention
class FindAttention(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['giveTool'], output_keys=['head_pose'])

    def execute(self, userdata):
        ## Call service to get the frame id of the hand centroid
        userdata.head_pose = rospy.wait_for_message("/openface2/head_pose", PoseStamped)
        rospy.loginfo('Executing state FindAttention')
        return 'giveTool'

# define state IK2
class IK2(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['foundIK'], input_keys=['head_pose'])

    def execute(self, userdata):
        ## Wait till IK not found. Change tolerance and call compute IK again
        ## Break go to pose into find pose and compute IK functions
        ## Make a different function with gripper pointing upwards and call it from here
        success = ctrl.go_to_handover_pose(userdata.head_pose)
        # if not success
        rospy.loginfo('Executing state IK2')
        return 'foundIK'

# # define state MoveGive
# class MoveGive(smach.State):
#     def __init__(self):
#         smach.State.__init__(self, outcomes=['reached'])

#     def execute(self, userdata):
#         rospy.loginfo('Executing state MoveGive')
#         return 'reached'

# define state ChangePID
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
                               transitions={'gotToolInput':'FINDTOOL'})
        smach.StateMachine.add('FINDTOOL', FindTool(),
                               transitions={'foundTool':'IK1', 'notfoundTool':'IDLE'})
        smach.StateMachine.add('IK1', IK1(),
                               transitions={'noIK':'stop','foundIK':'GRAB'})
        # smach.StateMachine.add('MOVE', Move(),
        #                        transitions={'reached':'GRAB'})
        smach.StateMachine.add('GRAB', Grab(),
                               transitions={'grabSuccess':'MOVEHOME2','grabFailure':'FINDTOOL'})
        smach.StateMachine.add('MOVEHOME2', MoveHome2(),
                               transitions={'reached':'ATTENTIONSEEKER'})
        # smach.StateMachine.add('ORIENTCAMERA', OrientCamera(),
        #                        transitions={'reached':'ATTENTIONSEEKER'})
        smach.StateMachine.add('ATTENTIONSEEKER', FindAttention(),
                               transitions={'giveTool':'IK2'})
        smach.StateMachine.add('IK2', IK2(),
                               transitions={'foundIK':'CHANGEPID'})
        # smach.StateMachine.add('MOVEGIVE', MoveGive(),
        #                        transitions={'reached':'CHANGEPID'})

        smach.StateMachine.add('CHANGEPID', ChangePID(),
                               transitions={'changed':'OPENGRIPPER', 'notchanged': 'CHANGEPID'})
        smach.StateMachine.add('OPENGRIPPER', OpenGripper(),
                               transitions={'opened':'IDLE'})



    # Execute SMACH plan
    outcome = sm.execute()
    sis.stop()





if __name__ == '__main__':
    main()
