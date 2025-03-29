# Import necessary modules
import moveit_commander
import geometry_msgs.msg 
from geometry_msgs.msg import Pose
import time
import sys
sys.path.append("/home/ros_ws")
from src.devel_packages.manipulation.src.moveit_class import MoveItPlanner
import yaml


# Create a MoveItPlanner object and start the MoveIt node
franka_moveit = MoveItPlanner()

franka_moveit.fa.reset_joints()

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

# Get the current robot position and add some Euler rotation
pose_goal = moveitcommander.get_current_pose().pose

# save the current position to a file as a yaml file
# with open("cup_detection_position.yaml", "w") as f:
#     yaml.dump({"position": {"x": pose_goal.position.x, "y": pose_goal.position.y, "z": pose_goal.position.z},
#                "orientation": {"x": pose_goal.orientation.x, "y": pose_goal.orientation.y, "z": pose_goal.orientation.z, "w": pose_goal.orientation.w}}, f)

#read the saved position from the yaml file 
with open("cup_detection_position.yaml", "r") as f:
    data = yaml.load(f, Loader=yaml.FullLoader)
    # print(data)


# set the pose goal based on the position read from the yaml file
pose_goal = Pose()
pose_goal.position.x = data["position"]["x"]
pose_goal.position.y = data["position"]["y"]
pose_goal.position.z = data["position"]["z"]
pose_goal.orientation.x = data["orientation"]["x"]
pose_goal.orientation.y = data["orientation"]["y"]
pose_goal.orientation.z = data["orientation"]["z"]
pose_goal.orientation.w = data["orientation"]["w"]

# print("pose_goal: ", pose_goal)

# # rotate the end effector by 90 degrees about the z axis
# # pose_goal = Pose()
# pose_goal.position.x = 0.5843781940153249
# pose_goal.position.y = 0.05791107711908864
# #pose_goal.position.z = 0.23098061041636195
# pose_goal.position.z += 0.2
# pose_goal.orientation.x = -0.9186984147774666
# pose_goal.orientation.y = 0.3942492534293267
# pose_goal.orientation.z = -0.012441904611284204 
# pose_goal.orientation.w = 0.020126567105018894

# Convert pose goal to the panda_hand frame (the frame that MoveIt uses)
pose_goal = franka_moveit.get_moveit_pose_given_frankapy_pose(pose_goal)

# plan a straight line motion to the goal
joints = franka_moveit.get_straight_plan_given_pose(pose_goal)
print(joints)
# print(plan)

# # execute the plan (uncomment after verifying plan on rviz)
franka_moveit.execute_plan(joints)
