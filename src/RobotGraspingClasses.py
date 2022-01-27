#!/usr/bin/python3
# -*- coding: utf-8 -*-

import rospy
import smach
import actionlib
from mycobot_320_moveit.msg import *
from grasp_pose_gpd.msg import *
from scipy.spatial.transform import Rotation as R
import numpy as np
import open3d as o3d
import os
import time, subprocess
from pymycobot.mycobot import MyCobot
from harvesters.core import Harvester
from harvesters_pipeline import policy
from pointcloudtransformation import rawdata_to_pcd_file

state_machine_path = os.path.abspath(os.path.join(__file__, *[os.pardir] * 2))
pcd_data_path = os.path.join(state_machine_path, 'pcd_data')
######################################################################## Initialization #############################################################################################################################
class Initialization(smach.State):
    """
        This class initializes the parameter for later use
    """

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'],
                                   input_keys=[],
                                   output_keys=[])

    def execute(self, userdata):

        global multi_move_client
        global grasp_client
        global move_home_client
        global singel_move_client

        multi_move_client = actionlib.SimpleActionClient('multi_move', MultiMoveAction)
        grasp_client = actionlib.SimpleActionClient('generate_grasps', GraspPoseAction)
        move_home_client = actionlib.SimpleActionClient('move_home', MoveHomeAction)
        singel_move_client = actionlib.SimpleActionClient('singel_move', SingelMoveAction)

        return 'succeeded'

######################################################################## GraspDetection #############################################################################################################################
class GraspDetection(smach.State):
    """
        This Class is used to detect and generate valid grasping pose.
        The class uses an actionlib grasp client which was created in 
        the initialization process. The class sends a goal to
        the actionlib grasp detection server containing the target pcd file directory.
    """
    
    def __init__(self):

        smach.State.__init__(self, outcomes=['detection_completed', 'detection_aborted', 'fault'], 
                                   input_keys=[], 
                                   output_keys=['approach_point_x', 'approach_point_y', 'approach_point_z', 'approach_point_ox', 
                                                'approach_point_oy', 'approach_point_oz', 'approach_point_ow', 'grasp_point_x', 
                                                'grasp_point_y', 'grasp_point_z', 'grasp_point_ox', 'grasp_point_oy', 'grasp_point_oz', 
                                                'grasp_point_ow'])


    def execute(self, userdata):
        try:
          # the client is initialized in the Initialization() class
          global grasp_client
          print("Waiting for the gpd server ...")
          grasp_client.wait_for_server()
          print("\n --- gpd server ready --- \n")

          # creating the goal object
          goal = GraspPoseGoal()
          # filling the goal attributes
          files = [x for x in os.listdir(pcd_data_path)]
          pcd_file = files[-1]
          pcd_file_path = os.path.join(pcd_data_path, pcd_file)
          goal.action_name = pcd_file_path
          # sending the goal to the server
          grasp_client.send_goal(goal)
          client_state = grasp_client.get_state()
          while client_state != 3: # not in [2,3,4,5,8]
              client_state = grasp_client.get_state()
              # ABORTED : 4
              if client_state == 4:
                  print('--- No grasp candidates found with a positive score ---\n')
                  return 'detection_aborted'

          # filling userdata infos about approach point and grasping point
          client_result = grasp_client.get_result()
          userdata.approach_point_x = [client_result.approach_pose_robot_link[0].pose.position.x, client_result.approach_pose_robot_link[1].pose.position.x]
          userdata.approach_point_y = [client_result.approach_pose_robot_link[0].pose.position.y, client_result.approach_pose_robot_link[1].pose.position.y]
          userdata.approach_point_z = [client_result.approach_pose_robot_link[0].pose.position.z, client_result.approach_pose_robot_link[1].pose.position.z]
          userdata.approach_point_ox = [client_result.approach_pose_robot_link[0].pose.orientation.x, client_result.approach_pose_robot_link[1].pose.orientation.x]
          userdata.approach_point_oy = [client_result.approach_pose_robot_link[0].pose.orientation.y, client_result.approach_pose_robot_link[1].pose.orientation.y]
          userdata.approach_point_oz = [client_result.approach_pose_robot_link[0].pose.orientation.z, client_result.approach_pose_robot_link[1].pose.orientation.z]
          userdata.approach_point_ow = [client_result.approach_pose_robot_link[0].pose.orientation.w, client_result.approach_pose_robot_link[1].pose.orientation.w]

          userdata.grasp_point_x = [client_result.grasp_candidate_robot_link[0].pose.position.x, client_result.grasp_candidate_robot_link[1].pose.position.x]
          userdata.grasp_point_y = [client_result.grasp_candidate_robot_link[0].pose.position.y, client_result.grasp_candidate_robot_link[1].pose.position.y]
          userdata.grasp_point_z = [client_result.grasp_candidate_robot_link[0].pose.position.z, client_result.grasp_candidate_robot_link[1].pose.position.z]
          userdata.grasp_point_ox = [client_result.grasp_candidate_robot_link[0].pose.orientation.x, client_result.grasp_candidate_robot_link[1].pose.orientation.x]
          userdata.grasp_point_oy = [client_result.grasp_candidate_robot_link[0].pose.orientation.y, client_result.grasp_candidate_robot_link[1].pose.orientation.y]
          userdata.grasp_point_oz = [client_result.grasp_candidate_robot_link[0].pose.orientation.z, client_result.grasp_candidate_robot_link[1].pose.orientation.z]
          userdata.grasp_point_ow = [client_result.grasp_candidate_robot_link[0].pose.orientation.w, client_result.grasp_candidate_robot_link[1].pose.orientation.w]

          #shows the point cloud with camera coordinate and grasping pose, so that user can make sure that the detection was done correctly
          #r = R.from_quat([client_result.grasp_candidate_camera_link.pose.orientation.x, client_result.grasp_candidate_camera_link.pose.orientation.y, client_result.grasp_candidate_camera_link.pose.orientation.z, client_result.grasp_candidate_camera_link.pose.orientation.w])
          #T_camera_to_grasp = np.eye(4)
          #T_camera_to_grasp[:3, :3] = r.as_matrix()
          #T_camera_to_grasp[0, 3] = client_result.grasp_candidate_camera_link.pose.position.x
          #T_camera_to_grasp[1, 3] = client_result.grasp_candidate_camera_link.pose.position.y
          #T_camera_to_grasp[2, 3] = client_result.grasp_candidate_camera_link.pose.position.z
          
          print('--- Grasp detection completed ---\n')
          return 'detection_completed'
        except Exception as e:
          print("EXEPTION : ", e)
          return 'fault'

######################################################################## PlanMoveToTarget #############################################################################################################################
class PlanMoveToTarget(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'],
                                   input_keys=['approach_point_x', 'approach_point_y', 'approach_point_z', 'approach_point_ox', 
                                                'approach_point_oy', 'approach_point_oz', 'approach_point_ow', 'grasp_point_x', 
                                                'grasp_point_y', 'grasp_point_z', 'grasp_point_ox', 'grasp_point_oy', 'grasp_point_oz', 
                                                'grasp_point_ow', 'planning_tries'],
                                   output_keys=['robot_approach_goal', 'robot_target_goal', 'planning_tries'])

    def execute(self, userdata):
        #Update the number of the planning trails
        userdata.planning_tries = userdata.planning_tries + 1

        # Get the positions and poses from user data
        approach_point_x = userdata.approach_point_x
        approach_point_y = userdata.approach_point_y
        approach_point_z = userdata.approach_point_z
        approach_point_ox = userdata.approach_point_ox
        approach_point_oy = userdata.approach_point_oy
        approach_point_oz = userdata.approach_point_oz
        approach_point_ow = userdata.approach_point_ow

        grasp_point_x = userdata.grasp_point_x
        grasp_point_y = userdata.grasp_point_y
        grasp_point_z = userdata.grasp_point_z
        grasp_point_ox = userdata.grasp_point_ox
        grasp_point_oy = userdata.grasp_point_oy
        grasp_point_oz = userdata.grasp_point_oz
        grasp_point_ow = userdata.grasp_point_ow

        approach_goal = robot_goals()
        target_goal = robot_goals()
        # A second pose for grasping, if robot arm can not reach the first pose because of the gripper direction, it can try with the second pose.
        if userdata.planning_tries == 2:
            approach_goal.x = approach_point_x[1]
            approach_goal.y = approach_point_y[1]
            approach_goal.z = approach_point_z[1]
            approach_goal.ox = approach_point_ox[1]
            approach_goal.oy = approach_point_oy[1]
            approach_goal.oz = approach_point_oz[1]
            approach_goal.ow =  approach_point_ow[1]

            target_goal.x = grasp_point_x[1]
            target_goal.y = grasp_point_y[1]
            target_goal.z = grasp_point_z[1]
            target_goal.ox = grasp_point_ox[1]
            target_goal.oy = grasp_point_oy[1]
            target_goal.oz = grasp_point_oz[1]
            target_goal.ow = grasp_point_ow[1]

        elif userdata.planning_tries == 1:
            approach_goal.x = approach_point_x[0]
            approach_goal.y = approach_point_y[0]
            approach_goal.z = approach_point_z[0]
            approach_goal.ox = approach_point_ox[0]
            approach_goal.oy = approach_point_oy[0]
            approach_goal.oz = approach_point_oz[0]
            approach_goal.ow =  approach_point_ow[0]

            target_goal.x = grasp_point_x[0]
            target_goal.y = grasp_point_y[0]
            target_goal.z = grasp_point_z[0]
            target_goal.ox = grasp_point_ox[0]
            target_goal.oy = grasp_point_oy[0]
            target_goal.oz = grasp_point_oz[0]
            target_goal.ow = grasp_point_ow[0]
        else:
            print('--- PlanMoveToTarget too many tries ---')
            return 'failed'
        print(approach_goal)
        userdata.robot_approach_goal = approach_goal
        userdata.robot_target_goal = target_goal
        print('--- PlanMoveToTarget completed ---')
        return 'succeeded'

######################################################################## MoveToTarget #############################################################################################################################
class MoveToTarget(smach.State):
    """
        This Class is used to move the robot arm.
        The class uses an actionlib movement client which was created in 
        the initialization process. The class sends a goal to
        the actionlib movement server containing the approach position and target position.
    """
    def __init__(self):

        smach.State.__init__(self, outcomes=['target_reached', 'target_not_reached'], input_keys=['robot_approach_goal', 'robot_target_goal'], output_keys=[])

    def execute(self, userdata):
        # the client is initialized in the Initialization() class
        global multi_move_client
        print("Waiting for the multi_move server ...")
        multi_move_client.wait_for_server()
        print("\n --- multi_move server ready --- \n")

        # creating the goal object
        goal = MultiMoveGoal()
        goal.targetPosition.append(userdata.robot_approach_goal)
        goal.targetPosition.append(userdata.robot_target_goal)
        # sending the goal to the server
        multi_move_client.send_goal(goal)

        client_state = multi_move_client.get_state()
        while client_state != 3: # not in [2,3,4,5,8]
            client_state = multi_move_client.get_state()
            # ABORTED : 4
            if client_state == 4:
                print('--- Target not reached try again ---')
                return 'target_not_reached'

        print('--- Multi Movement completed ---')
        return 'target_reached'

######################################################################## WaitForUser #############################################################################################################################
class WaitForUser(smach.State):
    """
        This class waits for the user input to start the process.
        
    """

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'], input_keys=[], output_keys=['planning_tries'])

    def execute(self, userdata):
        text = input("---type in enter to start--- \n")
        for f in os.listdir(pcd_data_path):
            os.remove(os.path.join(pcd_data_path, f))
        if text == "":
            print("start process")
            userdata.planning_tries = 0
            return 'succeeded'
        else:
            return 'failed'

######################################################################## MoveToHome #############################################################################################################################
class MoveToHome(smach.State):
    """
        This Class is used to move the robot arm to home position.
        The class uses an actionlib movement client which was created in 
        the initialization process.
    """
    def __init__(self):

        smach.State.__init__(self, outcomes=['target_reached'], input_keys=[], output_keys=[])

    def execute(self, userdata):
        # the client is initialized in the Initialization() class
        global move_home_client
        print("Waiting for the move_home server ...")
        move_home_client.wait_for_server()
        print("\n --- move_home server ready --- \n")

        # creating the goal object
        goal = MoveHomeGoal()
        goal.MoveHomeGoal = 'move_home'

        # sending the goal to the server
        move_home_client.send_goal(goal)

        client_state = move_home_client.get_state()
        while client_state != 3: # not in [2,3,4,5,8]
            client_state = move_home_client.get_state()

        print('--- Homing Movement completed ---')
        return 'target_reached'

######################################################################## GrasperClose #############################################################################################################################
class GrasperClose(smach.State):
    """
        This class is used to close the grasper.
        
    """

    def __init__(self):
        smach.State.__init__(self, outcomes=['closed'], input_keys=[], output_keys=[])

    def execute(self, userdata):
        port = subprocess.check_output(["echo -n /dev/ttyUSB*"], shell=True).decode()
        mycobot = MyCobot(port)
        time.sleep(3)
        mycobot.set_gripper_state(0, 100)
        return 'closed'

######################################################################## WaitForConfirm #############################################################################################################################
class WaitForConfirm(smach.State):
    """
        This class waits for the user input to continue the process.
        
    """

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'], input_keys=[], output_keys=[])

    def execute(self, userdata):
        text = input("---type in enter to start--- \n")
        if text == "":
            print("continue process")
            return 'succeeded'
        else:
            return 'failed'

######################################################################## GrasperOpen #############################################################################################################################
class GrasperOpen(smach.State):
    """
        This class is used to open the grasper.
        
    """

    def __init__(self):
        smach.State.__init__(self, outcomes=['opened'], input_keys=[], output_keys=[])

    def execute(self, userdata):
        port = subprocess.check_output(["echo -n /dev/ttyUSB*"], shell=True).decode()
        mycobot = MyCobot(port)
        time.sleep(3)
        mycobot.set_encoder(7, 3800)
        return 'opened'

######################################################################## MoveToReleasePoint #############################################################################################################################
class MoveToReleasePoint(smach.State):
    """
        This class is used to move the object to the release point.
        
    """

    def __init__(self):
        smach.State.__init__(self, outcomes=['target_reached'], input_keys=[], output_keys=[])

    def execute(self, userdata):
        # the client is initialized in the Initialization() class
        global singel_move_client
        print("Waiting for the singel_move server ...")
        singel_move_client.wait_for_server()
        print("\n --- singel_move server ready --- \n")

        # creating the goal object
        goal = SingelMoveGoal()
        target_goal = robot_goals()
        target_goal.x = -0.31842
        target_goal.y = -0.09015
        target_goal.z = 0.176
        target_goal.ox = -0.55876
        target_goal.oy = 0
        target_goal.oz = 0.82933
        target_goal.ow =  -0.00029
        goal.targetPosition = target_goal
        # sending the goal to the server
        singel_move_client.send_goal(goal)

        client_state = singel_move_client.get_state()
        while client_state != 3: # not in [2,3,4,5,8]
            client_state = singel_move_client.get_state()

        print('--- Singel Movement completed ---')
        return 'target_reached'

######################################################################## CaptureImage #############################################################################################################################
class CaptureImage(smach.State):
    """
        This class is used to capture the image and save it as point cloud file.
        
    """

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'], input_keys=[], output_keys=[])

    def execute(self, userdata):
        policy()
        rawdata_to_pcd_file()
        time.sleep(1)
        return 'succeeded'
