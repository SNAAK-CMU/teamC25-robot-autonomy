# Import necessary modules
import moveit_commander
import geometry_msgs.msg 
from geometry_msgs.msg import Pose
import time
import sys
sys.path.append("/home/ros_ws")
from src.devel_packages.manipulation.src.moveit_class import MoveItPlanner
import yaml
import tf.transformations as tf_transformations
import numpy as np
from autolab_core import RigidTransform


# Create a MoveItPlanner object and start the MoveIt node
franka_moveit = MoveItPlanner()

# Create a MoveGroupCommander for the "panda_arm" group
moveitcommander = moveit_commander.MoveGroupCommander("panda_arm")

# Create a PlanningSceneInterface object directly
scene = moveit_commander.PlanningSceneInterface()

# Define the box pose
box_pose = geometry_msgs.msg.PoseStamped()
box_pose.header.frame_id = "panda_link0"  # or the robot's base frame
box_pose.pose.position.x = 0.53
box_pose.pose.position.y = 0.0
box_pose.pose.position.z = 0.0

# Add the box to the planning scene
box_name = "my_box"
scene.add_box(box_name, box_pose, size=(0.57, 0.4, 0.15))  # dimensions: x, y, z

# Wait for the scene to update
time.sleep(2)

save_pos = False # set to True to save the current position to a yaml file

if save_pos:
    # Get the current robot position and add some Euler rotation
    pose_goal = franka_moveit.fa.get_pose()
    
    # Convert translation to a simple Python list
    translation = pose_goal.translation.tolist()  # convert to a Python list
    
    rotation_mat = pose_goal.rotation
    transformation_matrix = np.eye(4)
    transformation_matrix[:3, :3] = rotation_mat
    q = tf_transformations.quaternion_from_matrix(transformation_matrix)
    
    # Convert quaternion to a simple Python list
    q_list = q.tolist()  # convert to a Python list

    print("pose_goal: ", translation)
    print("pose_goal: ", q_list)
    
    # save both pose_goal.translation and q to a yaml file with the keys "position" and "orientation"
    # with open("cup_detection_position.yaml", "w") as f:
    with open("grasp_position.yaml", "w") as f:
    # with open("pre_pour.yaml", "w") as f:
        yaml.dump({
            "position": {"x": translation[0], "y": translation[1], "z": translation[2]},
            "orientation": {"x": q_list[0], "y": q_list[1], "z": q_list[2], "w": q_list[3]}
        }, f)

    
else:
    #######################################################
    #                  Reset Joints
    #######################################################
    
    franka_moveit.fa.open_gripper()
    franka_moveit.fa.reset_joints()

    #######################################################
    #               Pre Grasp Position
    #######################################################
    #read the saved position from the yaml file 
    with open("cup_detection_position.yaml", "r") as f:
        data = yaml.load(f, Loader=yaml.FullLoader)
        # print(data)
    f.close()
    # set the pose goal based on the position read from the yaml file
    pose_goal = Pose()
    pose_goal.position.x = data["position"]["x"]
    pose_goal.position.y = data["position"]["y"]
    pose_goal.position.z = data["position"]["z"]
    pose_goal.orientation.x = data["orientation"]["x"]
    pose_goal.orientation.y = data["orientation"]["y"]
    pose_goal.orientation.z = data["orientation"]["z"]
    pose_goal.orientation.w = data["orientation"]["w"]

    print("pose_goal: ", pose_goal)

    # Convert pose goal to the panda_hand frame (the frame that MoveIt uses)
    pose_goal = franka_moveit.get_moveit_pose_given_frankapy_pose(pose_goal)

    # plan a straight line motion to the goal
    joints = franka_moveit.get_straight_plan_given_pose(pose_goal)
    print(joints)
    # print(plan)

    # # execute the plan (uncomment after verifying plan on rviz)
    franka_moveit.execute_plan(joints)

    franka_moveit.fa.wait_for_skill()


    #######################################################
    #                  Grasp Position
    # #######################################################
    with open("grasp_position.yaml", "r") as f:
        data = yaml.load(f, Loader=yaml.FullLoader)
        # print(data)
    f.close()

    # set the pose goal based on the position read from the yaml file
    pose_goal = Pose()
    pose_goal.position.x = data["position"]["x"]
    pose_goal.position.y = data["position"]["y"]
    pose_goal.position.z = data["position"]["z"]
    pose_goal.orientation.x = data["orientation"]["x"]
    pose_goal.orientation.y = data["orientation"]["y"]
    pose_goal.orientation.z = data["orientation"]["z"]
    pose_goal.orientation.w = data["orientation"]["w"]

    print("pose_goal: ", pose_goal)

    # Convert pose goal to the panda_hand frame (the frame that MoveIt uses)
    pose_goal = franka_moveit.get_moveit_pose_given_frankapy_pose(pose_goal)

    # plan a straight line motion to the goal
    joints = franka_moveit.get_straight_plan_given_pose(pose_goal)
    print(joints)
    # print(plan)

    # # execute the plan (uncomment after verifying plan on rviz)
    franka_moveit.execute_plan(joints)

    franka_moveit.fa.goto_gripper(0.002, grasp=True, force=25)

    franka_moveit.fa.wait_for_skill()


    #######################################################
    #             Pre Pour Position
    #######################################################
    with open("pre_pour.yaml", "r") as f:
        data = yaml.load(f, Loader=yaml.FullLoader)
        # print(data)
    f.close()
    # set the pose goal based on the position read from the yaml file
    pose_goal = Pose()
    pose_goal.position.x = data["position"]["x"]
    pose_goal.position.y = data["position"]["y"]
    pose_goal.position.z = data["position"]["z"]
    pose_goal.orientation.x = data["orientation"]["x"]
    pose_goal.orientation.y = data["orientation"]["y"]
    pose_goal.orientation.z = data["orientation"]["z"]
    pose_goal.orientation.w = data["orientation"]["w"]

    print("pose_goal: ", pose_goal)

    # Convert pose goal to the panda_hand frame (the frame that MoveIt uses)
    pose_goal = franka_moveit.get_moveit_pose_given_frankapy_pose(pose_goal)

    # plan a straight line motion to the goal
    joints = franka_moveit.get_straight_plan_given_pose(pose_goal)
    print(joints)
    # print(plan)

    # # execute the plan (uncomment after verifying plan on rviz)
    franka_moveit.execute_plan(joints)

    franka_moveit.fa.wait_for_skill()












    # default_rotation = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])

    # # 3x3 rotation matrix to rotate about y-axis by 5 degrees
    # initial_pitch = 0.25 #0.25
    # additional_rotation = np.array([[np.cos(initial_pitch), 0, np.sin(initial_pitch)], [0, 1, 0], [-np.sin(initial_pitch), 0, np.cos(initial_pitch)]])
    # default_rotation = default_rotation @ additional_rotation

    # # # move to x, y, and z directly above the bin
    # # pre_pour_pose = RigidTransform(from_frame='franka_tool', to_frame='world') # TODO get transform to bottle and define it as tool frame
    # # # pre_pour_pose.translation = [0.3261, 0.012, 0.3947] # [0.3261, 0.012, 0.3447]
    # # pre_pour_pose.position = [0.45, 0.012, 0.350] # [0.3261, 0.012, 0.3447]
    # transformation_matrix = np.eye(4)
    # transformation_matrix[:3, :3] = default_rotation
    # q = tf_transformations.quaternion_from_matrix(transformation_matrix)

    # pose_goal = Pose()
    # pose_goal.position.x = 0.45
    # pose_goal.position.y = 0.012
    # pose_goal.position.z = 0.350
    # pose_goal.orientation.x = q[0]
    # pose_goal.orientation.y = q[1]
    # pose_goal.orientation.z = q[2]
    # pose_goal.orientation.w = q[3]

    # pre_pour_pose = pose_goal

    # print("pose_goal: ", pre_pour_pose)

    # # Convert pose goal to the panda_hand frame (the frame that MoveIt uses)
    # pose_goal = franka_moveit.get_moveit_pose_given_frankapy_pose(pre_pour_pose)

    # # plan a straight line motion to the goal
    # joints = franka_moveit.get_straight_plan_given_pose(pre_pour_pose)
    # print(joints)
    # # print(plan)

    # # # execute the plan (uncomment after verifying plan on rviz)
    # franka_moveit.execute_plan(joints)

    # franka_moveit.fa.goto_gripper(0.002, grasp=True, force=25)

    # franka_moveit.fa.wait_for_skill()

    # print('Moved to pre-pour pose')



