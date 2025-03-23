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
from traj_opt import solve_bead_pour, dynamics

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
    target_weight = 50
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
    Kp = 0.003 #0.05
    Kd = 0.0001
    Ki = 0.01  #0.5
    duration = np.inf
    default_rotation = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])

    # 3x3 rotation matrix to rotate about y-axis by 5 degrees
    initial_pitch = 0.25 #0.25
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

    print('Generating trajectory...')
    X, U, sucess, t_vec, dt, params = solve_bead_pour(start_mass=current_weight, end_mass=target_weight, verbose=True)
    if (not sucess):
        print('Failed to generate trajectory')
        exit(1)

    # Save the U's and X's
    np.save('U.npy', U)
    np.save('X.npy', X)
    Xm = np.array(X)
    Um = np.array(U)

    #desired pitch rate
    X_dot = np.zeros((len(Xm), 1))
    for i in range(len(Um) - 1):
        X_dot[i] = dynamics(params, Xm[i], Um[i])

    plt.figure()
    plt.plot(t_vec, Xm, label="State")
    plt.xlabel("Time (s)")
    plt.ylabel("Weight in cup (g)")
    plt.title("State Trajectory")
    plt.legend()
    plt.show()

    plt.figure()
    plt.plot(t_vec[:-1], Um, label="Control")
    plt.xlabel("Time (s)")
    plt.ylabel("Pitch (rad)")
    plt.ylim((-0.5, 0.3))
    plt.title("Controls")
    plt.legend()
    plt.show()
    print('Press any key to proceed...')
    keyboard.read_key()
    U_actual = np.zeros(len(Um))
    try:
        fa.goto_pose(pre_pour_pose, duration=duration, dynamic=True, buffer_time=1)
        init_time = rospy.Time.now().to_time()
        message_id = 1 
        prev_weight = 0
        prev_e = 0
        # pouring loop
        #while (current_weight < target_weight):
        for i in range(len(Um)):
            print('Pouring...')
            current_weight = scale.read_weight()
            # print("Scale weight: ", current_weight)
            if current_weight == -1:
                current_weight = prev_weight
            else:
                prev_weight = current_weight

            #print(f'Current weight: {current_weight} grams')
            # e = target_weight - current_weight
            # # e = e/target_weight
            # e_dot = (prev_e - e ) / dt # TODO improve accuracy of dt
            # e_total += e * dt
            # e_total = np.clip(e_total, 0.0, 0.5)
            # print(f'Error: {e}, Error derivative: {e_dot}')
            # new_pitch = -(Kp * e + Kd * e_dot + Ki * e_total)
            # pitch = (Kp * e + Kd * e_dot + Ki * e_total)
            #print("Pitch pre clip: ", pitch)
            # because of direction, e_totalto pour pitch needs to be negative
            #new_pitch_clipped = np.clip(new_pitch, -np.pi/3, 0) + initial_pitch
            #pitch = limit_angle_update(pitch, new_pitch_clipped)
            e = current_weight - Xm[i + 1][0]
            e_dot = (e - prev_e) / dt #- X_dot[i+1][0]
            pitch = Um[i][0] + Kp * e + (e_dot) * Kd
            U_actual[i] = pitch
            # pitch = np.clip(pitch, -np.pi/3, 0)
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

            prev_e = e


        
        print('Finished pouring')
        plt.figure()
        plt.plot(t_vec[:-1], U_actual, label="Actual Controls")
        plt.plot(t_vec[:-1], Um, label="Trajory Controls")
        plt.xlabel("Time (s)")
        plt.ylabel("Pitch (rad)")
        plt.ylim((-0.5, 0.3))
        plt.title("Actual and Trajectory Controls")
        plt.legend()
        plt.show()
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