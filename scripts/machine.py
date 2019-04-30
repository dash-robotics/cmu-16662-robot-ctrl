#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros

# define state Idle
class Idle(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['gotToolInput'], input_keys=['tool_id'], output_keys=['tool_id'])

    def execute(self, userdata):
        userdata.tool_id = -1
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
        smach.State.__init__(self, outcomes=['foundTool'], input_keys=['tool_id'])

    def execute(self, userdata):
        rospy.loginfo('Executing state FINDTOOL')
        print(userdata.tool_id)
        return 'foundTool'
        
# define state IK1
class IK1(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['noIK','foundIK'])

    def execute(self, userdata):
        rospy.loginfo('Executing state IK1')
        return 'foundIK'

# define state Move
class Move(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['reached'])

    def execute(self, userdata):
        rospy.loginfo('Executing state IK1')
        return 'reached'

# define state Grab
class Grab(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['grabSuccess','grabFailure'])

    def execute(self, userdata):
        rospy.loginfo('Executing state IK1')
        return 'grabSuccess'

# define state MoveHome2
class MoveHome2(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['reached'])

    def execute(self, userdata):
        rospy.loginfo('Executing state MoveHome2')
        return 'reached'

# define state OreintCamera
class OreintCamera(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['reached'])

    def execute(self, userdata):
        rospy.loginfo('Executing state OreintCamera')
        return 'reached'

# define state FindAttention
class FindAttention(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['giveTool'])

    def execute(self, userdata):
        rospy.loginfo('Executing state FindAttention')
        return 'giveTool'

# define state IK2
class IK2(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['foundIK'])

    def execute(self, userdata):
        rospy.loginfo('Executing state IK2')
        return 'foundIK'

# define state MoveGive
class MoveGive(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['reached'])

    def execute(self, userdata):
        rospy.loginfo('Executing state MoveGive')
        return 'reached'

# define state ChangePID
class ChangePID(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['reached'])

    def execute(self, userdata):
        rospy.loginfo('Executing state ChangePID')
        return 'reached'

# define state OpenGripper
class OpenGripper(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['reached'])

    def execute(self, userdata):
        rospy.loginfo('Executing state OpenGripper')
        return 'reached'

def main():
    rospy.init_node('attention_bot')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['stop'])
    sm.userdata.tool_id = -1

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('IDLE', Idle(), 
                               transitions={'gotToolInput':'FINDTOOL'})
        smach.StateMachine.add('FINDTOOL', FindTool(), 
                               transitions={'foundTool':'IK1'})
        smach.StateMachine.add('IK1', IK1(), 
                               transitions={'noIK':'stop','foundIK':'MOVE'})
        smach.StateMachine.add('MOVE', Move(), 
                               transitions={'reached':'GRAB'})
        smach.StateMachine.add('GRAB', Grab(), 
                               transitions={'grabSuccess':'MOVEHOME2','grabFailure':'FINDTOOL'})
        smach.StateMachine.add('MOVEHOME2', MoveHome2(), 
                               transitions={'reached':'ORIENTCAMERA'})
        smach.StateMachine.add('ORIENTCAMERA', OreintCamera(), 
                               transitions={'reached':'ATTENTIONSEEKER'})
        smach.StateMachine.add('ATTENTIONSEEKER', FindAttention(), 
                               transitions={'giveTool':'IK2'})
        smach.StateMachine.add('IK2', IK2(), 
                               transitions={'foundIK':'MOVEGIVE'})
        smach.StateMachine.add('MOVEGIVE', MoveGive(), 
                               transitions={'reached':'CHANGEPID'})
        smach.StateMachine.add('CHANGEPID', ChangePID(), 
                               transitions={'reached':'OPENGRIPPER'})
        smach.StateMachine.add('OPENGRIPPER', OpenGripper(), 
                               transitions={'reached':'IDLE'})
        

    # Execute SMACH plan
    outcome = sm.execute()



if __name__ == '__main__':
    main()