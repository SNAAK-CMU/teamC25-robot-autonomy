#!/usr/bin/env python3

import numpy as np
from weighing_scale import WeighingScale
import rospy
import keyboard
import time
import matplotlib.pyplot as plt

from autolab_core import RigidTransform
from frankapy import FrankaArm, SensorDataMessageType
from frankapy import FrankaConstants as FC
from frankapy.proto_utils import sensor_proto2ros_msg, make_sensor_group_msg

from frankapy.proto import PosePositionSensorMessage, ShouldTerminateSensorMessage, CartesianImpedanceSensorMessage
from franka_interface_msgs.msg import SensorDataGroup

from frankapy.utils import min_jerk, min_jerk_weight
from traj_opt import solve_bead_pour

def limit_angle_update(original_angle, new_angle, max_angle_change=0.01):
    angle_diff = new_angle - original_angle
    limited_diff = np.clip(angle_diff, -max_angle_change, max_angle_change)
    return original_angle + limited_diff

if __name__ == '__main__':
    fa = FrankaArm()

    # Changing tool frame to cup edge to ensure rotation about cup edge
    cup_edge_frame = RigidTransform(from_frame='franka_tool', to_frame='franka_tool_base') # TODO get transform to bottle and define it as tool frame
    cup_edge_frame.translation = [0.110, 0, 0.030] # [0.3261, 0.012, 0.3447]
    cup_edge_frame.rotation = np.eye(3)
    fa.set_tool_delta_pose(cup_edge_frame)

    # print("translation:")
    # print(fa.get_pose().translation)

    scale = WeighingScale()
    target_weight = scale.read_weight_on_key()
    target_weight = 57.9
    print(f"Target weight: {target_weight} grams")
    print(f"Remove object and press 'p' to move to pre-pour pose...")
    while True:
        if keyboard.is_pressed('p'):
            break
    
    current_weight = scale.read_weight()

    prev_e = target_weight - current_weight

    # Parameters for PID controller
    e_total = 0
    dt = 0.05
    Kp = 0.01 #0.05
    Kd = 0.05
    Ki = 0.01  #0.5
    duration = np.inf
    default_rotation = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])

    # 3x3 rotation matrix to rotate about y-axis by 5 degrees
    initial_pitch = 0.20 #0.25
    additional_rotation = np.array([[np.cos(initial_pitch), 0, np.sin(initial_pitch)], [0, 1, 0], [-np.sin(initial_pitch), 0, np.cos(initial_pitch)]])
    default_rotation = default_rotation @ additional_rotation
    pitch = initial_pitch

    # Grasp bottle
    #TODO goto bottle pick pose 

    # move to x, y, and z directly above the bin
    pre_pour_pose = RigidTransform(from_frame='franka_tool', to_frame='world') # TODO get transform to bottle and define it as tool frame
    # pre_pour_pose.translation = [0.3261, 0.012, 0.3947] # [0.3261, 0.012, 0.3447]
    pre_pour_pose.translation = [0.45, 0.012, 0.350] # [0.3261, 0.012, 0.3447]
    pre_pour_pose.rotation = default_rotation
    fa.goto_pose(pre_pour_pose)
    print('Moved to pre-pour pose')


    # close gripper
    fa.open_gripper()
    # wait for keyboard input
    print('Press any key to close gripper')
    keyboard.read_key()
    fa.goto_gripper(0.002, grasp=True, force=25)
    print('Gripped bottle')

    # define publisher
    rospy.loginfo('Initializinge_total Sensor Publisher')
    pub = rospy.Publisher(FC.DEFAULT_SENSOR_PUBLISHER_TOPIC, SensorDataGroup, queue_size=1000)
    rate = rospy.Rate(1 / dt)



    print('Press any key to proceed...')
    keyboard.read_key()
    try:
        fa.goto_pose(pre_pour_pose, duration=duration, dynamic=True, buffer_time=1)
        init_time = rospy.Time.now().to_time()
        message_id = 1 
        prev_weight = target_weight - 10
        # pouring loop
        while (current_weight < target_weight):
            print('Pouring...')
            current_weight = scale.read_weight()
            # print("Scale weight: ", current_weight)
            if current_weight == -1:
                current_weight = prev_weight
            else:
                prev_weight = current_weight

            if current_weight >= prev_weight:

            
            print(f'Pitch: {pitch}')
            rotation = default_rotation @ np.array([[np.cos(pitch), 0, np.sin(pitch)], [0, 1, 0], [-np.sin(pitch), 0, np.cos(pitch)]])
            
            pour_pose = RigidTransform(from_frame='franka_tool', to_frame='world')
            pour_pose.rotation = rotation
            pour_pose.translation = pre_pour_pose.translation

            # subtract the pour_pose with [0.110, 0, 0.030] 
            pour_pose.translation -= np.array([0.110, 0, 0.030])


            timestamp = rospy.Time.now().to_time() - init_time
            traj_gen_proto_msg = PosePositionSensorMessage(
                id=message_id, timestamp=timestamp, 
                position=pour_pose.translation, quaternion=pour_pose.quaternion #TODO check if quaternion works
            )
            ros_msg = make_sensor_group_msg(
                trajectory_generator_sensor_msg=sensor_proto2ros_msg(traj_gen_proto_msg, SensorDataMessageType.POSE_POSITION))
                
            # rospy.loginfo('Publishing: ID {}'.format(traj_gen_proto_msg.id))
            pub.publish(ros_msg)
            rate.sleep()

            #prev_e = e


        
        print('Finished pouring')

    except Exception as e:  
        print(f'Error: {e}')
    finally:#!/usr/bin/env python3
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

        pre_pour_pose = RigidTransform(from_frame='franka_tool', to_frame='world') # TODO get transform to bottle and define it as tool frame
        # pre_pour_pose.translation = [0.3261, 0.012, 0.3947] # [0.3261, 0.012, 0.3447]
        pre_pour_pose.translation = [0.45, 0.012, 0.350] # [0.3261, 0.012, 0.3447]
        pre_pour_pose.rotation = default_rotation
        fa.goto_pose(pre_pour_pose)
        print('Moved to pre-pour pose')