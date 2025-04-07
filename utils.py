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



def limit_angle_update(original_angle, new_angle, max_angle_change=0.01):
    angle_diff = new_angle - original_angle
    limited_diff = np.clip(angle_diff, -max_angle_change, max_angle_change)
    return original_angle + limited_diff

def pour_beads(fa, Xm, Um, t_vec, scale, at_pre_pour = False, dt=0.05, verbose=True):
    Kp = 0.003 #0.05
    Kd = 0.0001
    Ki = 0.01 
    U_actual = np.zeros(len(Um))

    cup_edge_frame = RigidTransform(from_frame='franka_tool', to_frame='franka_tool_base')
    cup_edge_frame.translation = [0.110, 0, 0.030]
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
    try:
        fa.goto_pose(pre_pour_pose, duration=t_vec[-1], dynamic=True, buffer_time=1)
        init_time = rospy.Time.now().to_time()
        message_id = 1 
        prev_weight = 0
        prev_e = 0
        
        # pouring loop
        print('Pouring...')

        for i in range(len(Um)):
            current_weight = scale.read_weight()

            if current_weight == -1:
                current_weight = prev_weight
            else:
                prev_weight = current_weight

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
            pour_pose.translation -= np.array([0.110, 0, 0.030])


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


        
        print('Finished pouring')
        if verbose:
            plt.figure()
            plt.plot(t_vec[:-1], U_actual, label="Actual Controls")
            plt.plot(t_vec[:-1], Um, label="Nominal Controls")
            plt.xlabel("Time (s)")
            plt.ylabel("Pitch (rad)")
            plt.ylim((-0.5, 0.3))
            plt.title("Actual and Nominal Controls")
            plt.legend()
            plt.show()
    except Exception as e:  
        print(f'Error: {e}')
    finally:
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
    target_weight = 50
    print(f"Target weight: {target_weight} grams")
    print("Place cup/bowl to catch beads on scale...")
    current_weight = scale.read_weight_on_key()
    return target_weight, current_weight
