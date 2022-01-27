#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from RobotGraspingClasses import *

def main():
    # This is used to give a title to the terminal window
    sys.stdout.write('\33]0;State machine node\a')
    sys.stdout.flush()

    rospy.init_node('RobotGrasping_SM')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['finished'])

    with sm:
        # Add states to the container
        smach.StateMachine.add('Init', Initialization(),
                               transitions={'succeeded': 'WaitForUser'}, remapping={})

        smach.StateMachine.add('WaitForUser', WaitForUser(),
                               transitions={'succeeded': 'CaptureImage', 'failed': 'WaitForUser'}, remapping={})
        
        smach.StateMachine.add('CaptureImage', CaptureImage(),
                               transitions={'succeeded': 'GraspDetection'}, remapping={})                       

        smach.StateMachine.add('GraspDetection', GraspDetection(),
                               transitions={'detection_completed': 'PlanMoveToTarget', 'detection_aborted': 'finished', 'fault': 'GraspDetection'}, 
                               remapping={})

        smach.StateMachine.add('PlanMoveToTarget', PlanMoveToTarget(),
                               transitions={'succeeded': 'MoveToTarget', 'failed': 'WaitForUser'}, remapping={})

        smach.StateMachine.add('MoveToTarget', MoveToTarget(),
                               transitions={'target_reached': 'GrasperClose', 'target_not_reached': 'PlanMoveToTarget'}, remapping={})

        smach.StateMachine.add('GrasperClose', GrasperClose(),
                               transitions={'closed': 'WaitForCloseConfirm'}, remapping={})

        smach.StateMachine.add('WaitForCloseConfirm', WaitForConfirm(),
                               transitions={'succeeded': 'MoveToHomeAsWayPoint', 'failed': 'WaitForCloseConfirm'}, remapping={})

        smach.StateMachine.add('MoveToHomeAsWayPoint', MoveToHome(),
                               transitions={'target_reached': 'MoveToReleasePoint'}, remapping={})

        smach.StateMachine.add('MoveToReleasePoint', MoveToReleasePoint(),
                               transitions={'target_reached': 'GrasperOpen'}, remapping={})

        smach.StateMachine.add('GrasperOpen', GrasperOpen(),
                               transitions={'opened': 'WaitForOpenConfirm'}, remapping={})

        smach.StateMachine.add('WaitForOpenConfirm', WaitForConfirm(),
                               transitions={'succeeded': 'MoveToHome', 'failed': 'WaitForOpenConfirm'}, remapping={})
        
        smach.StateMachine.add('MoveToHome', MoveToHome(),
                               transitions={'target_reached': 'WaitForUser'}, remapping={})


    # Create and start the introspection server
    #sis = smach_ros.IntrospectionServer('server_name', sm, '/StartRobotGraspingProcess')
    #sis.start()

    # Execute SMACH plan
    outcome = sm.execute()
    # Wait for ctrl-c to stop the application
    rospy.spin()
    #sis.stop()


if __name__ == '__main__':
    main()