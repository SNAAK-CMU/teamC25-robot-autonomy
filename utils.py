#!/usr/bin/env python3

import numpy as np
import rospy
import matplotlib.pyplot as plt

from autolab_core import RigidTransform
from frankapy import SensorDataMessageType
from frankapy import FrankaConstants as FC
from frankapy.proto_utils import sensor_proto2ros_msg, make_sensor_group_msg

from frankapy.proto import PosePositionSensorMessage, ShouldTerminateSensorMessage
from franka_interface_msgs.msg import SensorDataGroup
import cv2
import os
import yaml
import tf.transformations as tft
import geometry_msgs.msg
from src.devel_packages.manipulation.src.moveit_class import MoveItPlanner
from autolab_core import RigidTransform


def limit_angle_update(original_angle, new_angle, max_angle_change=0.01):
    angle_diff = new_angle - original_angle
    limited_diff = np.clip(angle_diff, -max_angle_change, max_angle_change)
    return original_angle + limited_diff

def pour_beads(fa, Xm, Um, t_vec, scale, at_pre_pour = False, dt=0.05, verbose=True):
    Kp = 0.003 #0.05
    Kd = 0.0001
    Ki = 0.01 
    U_actual = np.zeros(len(Um))
    X_actual = np.zeros(len(Xm))
    cup_edge_frame = RigidTransform(from_frame='franka_tool', to_frame='franka_tool_base')
    cup_edge_frame.translation = [0.09, 0, 0.11]
    cup_edge_frame.rotation = np.eye(3)
    fa.set_tool_delta_pose(cup_edge_frame)

    initial_pitch = 0.25 
    default_rotation = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
    additional_rotation = np.array([[np.cos(initial_pitch), 0, np.sin(initial_pitch)], [0, 1, 0], [-np.sin(initial_pitch), 0, np.cos(initial_pitch)]])
    default_rotation = default_rotation @ additional_rotation
    pitch = initial_pitch

    # move to x, y, and z directly above the cup
    pre_pour_pose = RigidTransform(from_frame='franka_tool', to_frame='world')
    pre_pour_pose.translation = [0.45, 0.012, 0.350] # [0.3261, 0.012, 0.3447]
    pre_pour_pose.rotation = default_rotation

    if not at_pre_pour:
        fa.goto_pose(pre_pour_pose)
        print('Moved to pre-pour pose')

    rospy.loginfo('Initializinge_total Sensor Publisher')
    pub = rospy.Publisher(FC.DEFAULT_SENSOR_PUBLISHER_TOPIC, SensorDataGroup, queue_size=1000)
    rate = rospy.Rate(1 / dt)
    #try:
    duration = np.inf
    fa.goto_pose(pre_pour_pose, duration=duration, dynamic=True, buffer_time=1)
    init_time = rospy.Time.now().to_time()
    message_id = 1 
    prev_weight = 0
    prev_e = 0
    input('Press Enter to start pouring...')
    # pouring loop
    print('Pouring...')

    for i in range(len(Um)):
        current_weight = scale.read_weight()
        
        if current_weight == -1:
            current_weight = prev_weight
        else:
            prev_weight = current_weight

        X_actual[i] = current_weight

        e = current_weight - Xm[i + 1][0]
        e_dot = (e - prev_e) / dt
        pitch = Um[i][0] + Kp * e + (e_dot) * Kd
        U_actual[i] = pitch

        print(f'Pitch: {pitch}')
        rotation = default_rotation @ np.array([[np.cos(pitch), 0, np.sin(pitch)], [0, 1, 0], [-np.sin(pitch), 0, np.cos(pitch)]])
        
        pour_pose = RigidTransform(from_frame='franka_tool', to_frame='world')
        pour_pose.rotation = rotation
        pour_pose.translation = pre_pour_pose.translation

        # subtract the pour_pose with [0.110, 0, 0.030] 
        pour_pose.translation += np.array([-0.09, 0, 0.11])
        # pour_pose.translation -= np.array([0.110, 0, 0.030])


        timestamp = rospy.Time.now().to_time() - init_time
        traj_gen_proto_msg = PosePositionSensorMessage(
            id=message_id, timestamp=timestamp, 
            position=pour_pose.translation, quaternion=pour_pose.quaternion #TODO check if quaternion works
        )
        ros_msg = make_sensor_group_msg(
            trajectory_generator_sensor_msg=sensor_proto2ros_msg(traj_gen_proto_msg, SensorDataMessageType.POSE_POSITION))
            
        pub.publish(ros_msg)
        rate.sleep()

        prev_e = e
    
    X_actual[-1] = scale.read_weight()
    
    print('Finished pouring')
    # if verbose:
    plt.figure()
    plt.plot(t_vec[:-1], U_actual, label="Actual Controls")
    plt.plot(t_vec[:-1], Um, label="Nominal Controls")
    plt.xlabel("Time (s)")
    plt.ylabel("Pitch (rad)")
    plt.ylim((-0.5, 0.3))
    plt.title("Actual and Nominal Controls")
    plt.legend()
    plt.show()
    plt.savefig('actual_nominal_controls.png')
    plt.figure()
    plt.plot(t_vec[:], Xm[:], label="Reference State")
    plt.plot(t_vec[:], X_actual[:], label="Actual State")
    plt.xlabel("Time (s)")
    plt.ylabel("Weight of beads (grams)")
    # plt.ylim((-0.5, 0.3))
    plt.title("Actual and Reference State")
    plt.legend()
    plt.show()
    plt.savefig('actual_reference_state.png')

    #except Exception as e:  
        # print(f'Error: {e}')
    #finally:
        # Stop the skill
        # Alternatively can call fa.stop_skill()
    term_proto_msg = ShouldTerminateSensorMessage(timestamp=rospy.Time.now().to_time() - init_time, should_terminate=True)
    ros_msg = make_sensor_group_msg(
        termination_handler_sensor_msg=sensor_proto2ros_msg(
            term_proto_msg, SensorDataMessageType.SHOULD_TERMINATE)
        )
    pub.publish(ros_msg)
    fa.wait_for_skill()
    rospy.loginfo('Done')

    pre_pour_pose = RigidTransform(from_frame='franka_tool', to_frame='world')
    pre_pour_pose.translation = [0.45, 0.012, 0.350] # [0.3261, 0.012, 0.3447]
    pre_pour_pose.rotation = default_rotation
    fa.goto_pose(pre_pour_pose)
    print('Moved to pre-pour pose')

def get_weights(scale):
    print("Place target weight object on scale...")
    target_weight = scale.read_weight_on_key()
    # target_weight = 60.0
    print(f"Target weight: {target_weight} grams")
    print("Place cup/bowl to catch beads on scale...")
    current_weight = scale.read_weight_on_key()
    return target_weight, current_weight

def get_pickup_pixels(img, verbose=False):
    # --- Mask everything except the ROI ---
    mask = np.zeros(img.shape[:2], dtype=np.uint8)  # Black mask same size as image

    # 🔧 Define your rectangle here (x1, y1, x2, y2)
    x1, y1 = 350, 90  # Top-left corner
    x2, y2 = 700, 340  # Bottom-right corner

    # Fill the ROI with white (allowed area)
    mask[y1:y2, x1:x2] = 255

    # Apply the mask to the image
    img = cv2.bitwise_and(img, img, mask=mask)
    if verbose:
        cv2.imshow("Masked Image", img)
        cv2.waitKey(0)

    # --- Preprocess the masked image ---
    img = cv2.GaussianBlur(img, (5, 5), 0)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    
    lower_red1 = np.array([0, 120, 70])
    upper_red1 = np.array([10, 255, 255])
    
    lower_red2 = np.array([170, 120, 70])
    upper_red2 = np.array([180, 255, 255])
    
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = cv2.bitwise_or(mask1, mask2)
    
    if verbose:
        cv2.imshow("Red Mask (ROI Applied)", mask)
        cv2.waitKey(0)

    if mask.shape[0] > 500:
        mask[500:, :] = 0

    _, thresholded = cv2.threshold(mask, 127, 255, cv2.THRESH_BINARY)
    contours, _ = cv2.findContours(thresholded, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    min_area = 1000
    max_area = 9000
    tape_contour = None
    for contour in contours:
        area = cv2.contourArea(contour)
        if min_area < area < max_area:
            tape_contour = contour
            break

    if tape_contour is None:
        raise Exception("Cup not found")

    M = cv2.moments(tape_contour)
    cX = int(M["m10"] / M["m00"])
    cY = int(M["m01"] / M["m00"])
    
    cv2.circle(img, (cX, cY), 7, (255, 255, 0), -1)
    cv2.drawContours(img, [tape_contour], -1, (255, 0, 255), 2)

    if verbose:
        cv2.imshow("Final Detection", img)
        cv2.waitKey(0)

    return cX, cY


def transform_to_matrix(transform):
    translation = transform.transform.translation
    rotation = transform.transform.rotation
    
    rotation_matrix = tft.quaternion_matrix([rotation.x, rotation.y, rotation.z, rotation.w])

    T_matrix = np.eye(4)
    T_matrix[:3, :3] = rotation_matrix[:3, :3]
    T_matrix[:3, 3] = [translation.x, translation.y, translation.z]
    
    return T_matrix

def get_pickup_location(rgb_img, depth_img, T, K, verbose=False):
    pixelX, pixelY = get_pickup_pixels(rgb_img, verbose=verbose)
    pixel = np.array([pixelX, pixelY, 1.0])
    depth = depth_img[pixelY, pixelX]
    depth = depth / 1000.0
    if depth == 0:
        raise Exception("Depth is 0")
    
    cam_coords = depth * np.linalg.inv(K) @ pixel
    if verbose:
        print(f"Depth: {depth}")
        print(T)
        print(f'camera_coord:{cam_coords}')
    X, Y, Z, W = T @ np.concatenate((cam_coords, [1]))
    X = X / W
    Y = Y / W
    Z = Z / W
    X += 0.042 # Assume looking head on, since we detect front of cup, pick up point is at center
    return X, Y, Z

def add_collision_boxes(franka_moveit):
    weighing_scale = geometry_msgs.msg.PoseStamped()
    weighing_scale.header.frame_id = "panda_link0"  # or the robot's base frame
    weighing_scale.pose.position.x = 0.53
    weighing_scale.pose.position.y = 0.0
    weighing_scale.pose.position.z = 0.15/2


    pickup_area = geometry_msgs.msg.PoseStamped()
    pickup_area.header.frame_id = "panda_link0"  # or the robot's base frame
    pickup_area.pose.position.x = 0.7
    pickup_area.pose.position.y = -0.3
    pickup_area.pose.position.z = 0.2

    # Define the walls
    left_wall = geometry_msgs.msg.PoseStamped()
    left_wall.header.frame_id = "panda_link0"  # or the robot's base frame
    left_wall.pose.position.x = 0.15
    left_wall.pose.position.y = 0.6
    left_wall.pose.position.z = 0.6

    right_wall = geometry_msgs.msg.PoseStamped()
    right_wall.header.frame_id = "panda_link0"  # or the robot's base frame
    right_wall.pose.position.x = 0.15
    right_wall.pose.position.y = -0.6
    right_wall.pose.position.z = 0.6

    back_wall = geometry_msgs.msg.PoseStamped()
    back_wall.header.frame_id = "panda_link0"  # or the robot's base frame
    back_wall.pose.position.x = -0.41
    back_wall.pose.position.y = 0.0
    back_wall.pose.position.z = 0.5

    bottom_wall = geometry_msgs.msg.PoseStamped()
    bottom_wall.header.frame_id = "panda_link0"  # or the robot's base frame
    bottom_wall.pose.position.x = 0.2
    bottom_wall.pose.position.y = 0.0
    bottom_wall.pose.position.z = 0.005

    # Add the box to the planning scene
    franka_moveit.add_box("weighing_scale", weighing_scale, size=(0.57, 0.4, 0.15))  # dimensions: x, y, z
    franka_moveit.add_box("pickup_area", pickup_area, size=(0.3, 0.2, 0.4))  # dimensions: x, y, z

    franka_moveit.add_box("left_wall", left_wall, size=(1.2, 0.01, 1.1))  # dimensions: x, y, z
    franka_moveit.add_box("right_wall", right_wall, size=(1.2, 0.01, 1.1))  # dimensions: x, y, z
    franka_moveit.add_box("back_wall", back_wall, size=(0.01, 1, 1.1))  # dimensions: x, y, z
    franka_moveit.add_box("bottom_wall", bottom_wall, size=(1.2, 1, 0.01)) 

def move_to_pre_pickup_location(franka_moveit):
    with open("cup_detection_position.yaml", "r") as f:
        data = yaml.load(f, Loader=yaml.FullLoader)
        # print(data)
    f.close()
    # set the pose goal based on the position read from the yaml file
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.position.x = data["position"]["x"]
    pose_goal.position.y = data["position"]["y"] + 0.05
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

def move_to_pre_pour_location(franka_moveit):
    with open("pre_pour.yaml", "r") as f:
        data = yaml.load(f, Loader=yaml.FullLoader)
        # print(data)s
    f.close()
    # set the pose goal based on the position read from the yaml file
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.position.x = data["position"]["x"]
    pose_goal.position.y = data["position"]["y"] + 0.05
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

def pickup(fa, X, Y, Z):
    default_rotation = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
    initial_pitch = np.pi / 2
    additional_rotation = np.array([[np.cos(initial_pitch), 0, np.sin(initial_pitch)], [0, 1, 0], [-np.sin(initial_pitch), 0, np.cos(initial_pitch)]])

    # move to the pre-pickup pose
    pickup_pose = RigidTransform(from_frame='franka_tool', to_frame='world')
    pickup_pose.translation = [X - 0.1, Y, Z]
    pickup_pose.rotation = default_rotation@additional_rotation
    fa.goto_pose(pickup_pose, duration=6)

    # move to pickup
    pickup_pose = RigidTransform(from_frame='franka_tool', to_frame='world')
    pickup_pose.translation = [X, Y, Z]
    pickup_pose.rotation = default_rotation@additional_rotation
    fa.goto_pose(pickup_pose, duration=6)
    print('Moved to pickup pose')

if __name__ == "__main__":
    rotation = np.array([0.01804096127439402, -0.006587662398836236, -0.6969376828282071, 0.7168744608887])

    # Quaternion for 90 degrees around the Y-axis
    q_rotate_y_90 = np.array([0.7071, 0, -0.7071, 0])

    # Multiply the existing quaternion by the Y-rotation quaternion
    adjusted_rotation = tft.quaternion_multiply(q_rotate_y_90, rotation)

    # Adjusted pose with updated rotation
    adjusted_pose = {
        'translation': {
            'x': -0.02330821724774943,
            'y': -0.007796542246582973,
            'z': -0.12652063647418807
        },
        'rotation': {
            'x': adjusted_rotation[0],
            'y': adjusted_rotation[1],
            'z': adjusted_rotation[2],
            'w': adjusted_rotation[3]
        }
    }

    # Print adjusted pose
    print("Adjusted Pose:")
    print(adjusted_pose)
