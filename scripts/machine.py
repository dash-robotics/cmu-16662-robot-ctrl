#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros

from controller import Controller
from geometry_msgs.msg import PoseStamped

# Initialize controller
ctrl = Controller()
# define state Idle
class Idle(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['gotToolInput'], input_keys=['tool_id'], output_keys=['tool_id'])

    def execute(self, userdata):
        userdata.tool_id = -1
        ctrl.set_camera_angles(ctrl.HOME_POS_CAMERA_01)
        ctrl.set_arm_joint_angles(ctrl.HOME_POS_MANIPULATOR_01)
        rospy.loginfo('Executing state IDLE')
        while(True):
            # TODO: Ask for Input
            userdata.tool_id = input("Enter Tool ID you want:")
            if(isinstance(userdata.tool_id, int) and userdata.tool_id>0 and userdata.tool_id<4):
                break

        # Return success
        return 'gotToolInput'



# define state FindTool
class FindTool(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['foundTool'], input_keys=['tool_id'], output_keys=['frame_name'])

    def execute(self, userdata):
        rospy.loginfo('Executing state FINDTOOL')
        print(userdata.tool_id)
        ## TODO: change the name of the variable frame_name
        userdata.frame_name = "random"
        return 'foundTool'

# define state IK1
class IK1(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['noIK','foundIK'], input_keys=['frame_name'])

    def execute(self, userdata):
        success = ctrl.go_to_pose(userdata.frame_name)
        rospy.loginfo('Executing state IK1')
        return 'foundIK' if success else 'noIK'

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
        smach.State.__init__(self, outcomes=['grabSuccess','grabFailure'])

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
        posestamped = rospy.wait_for_message("/openface2/head_pose", PoseStamped)
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
        success = ctrl.go_to_pose(userdata.head_pose)
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
        smach.State.__init__(self, outcomes=['changed'])

    def execute(self, userdata):
        ## Service call to change the PID values
        rospy.loginfo('Executing state ChangePID')
        return 'changed'

# define state OpenGripper
class OpenGripper(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['opened'])

    def execute(self, userdata):
        ## Detect when to Open gripper
        self.open_gripper()
        rospy.loginfo('Executing state OpenGripper')
        return 'opened'

def main():
    rospy.init_node('attention_bot')


    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['stop'])
    sm.userdata.tool_id = -1
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()
    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('IDLE', Idle(),
                               transitions={'gotToolInput':'FINDTOOL'})
        smach.StateMachine.add('FINDTOOL', FindTool(),
                               transitions={'foundTool':'IK1'})
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
                               transitions={'changed':'OPENGRIPPER'})
        smach.StateMachine.add('OPENGRIPPER', OpenGripper(),
                               transitions={'opened':'IDLE'})



    # Execute SMACH plan
    outcome = sm.execute()
    sis.stop()





if __name__ == '__main__':
    main()
