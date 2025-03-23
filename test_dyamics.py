import numpy as np
from weighing_scale import WeighingScale
import rospy
import keyboard
import time

from autolab_core import RigidTransform
from frankapy import FrankaArm, SensorDataMessageType
from frankapy import FrankaConstants as FC
from frankapy.proto_utils import sensor_proto2ros_msg, make_sensor_group_msg

from frankapy.proto import PosePositionSensorMessage, ShouldTerminateSensorMessage, CartesianImpedanceSensorMessage
from franka_interface_msgs.msg import SensorDataGroup

from frankapy.utils import min_jerk, min_jerk_weight


TARGET_PITCH = -0.05

EMPTY_FORCE_TORQUE = np.array([ 0.80408604, 0.6120421, 2.38530426, -0.06314893, -0.11238805, 0.23470744])



def define_cup_edge_as_ee(fa):
    cup_edge_frame = RigidTransform(from_frame='franka_tool', to_frame='franka_tool_base') # TODO get transform to bottle and define it as tool frame
    cup_edge_frame.translation = [0.110, 0, 0.030] # [0.3261, 0.012, 0.3447]
    cup_edge_frame.rotation = np.eye(3)
    fa.set_tool_delta_pose(cup_edge_frame)

def main():
    fa = FrankaArm()
    define_cup_edge_as_ee(fa)

    fa.open_gripper()
    # wait for keyboard input

    initial_pitch = 0.20
    default_rotation = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
    additional_rotation = np.array([[np.cos(initial_pitch), 0, np.sin(initial_pitch)], [0, 1, 0], [-np.sin(initial_pitch), 0, np.cos(initial_pitch)]])
    default_rotation = default_rotation @ additional_rotation
    pitch = initial_pitch

    pre_pour_pose = RigidTransform(from_frame='franka_tool', to_frame='world')
    pre_pour_pose.translation = [0.450, 0.012, 0.350]
    pre_pour_pose.rotation = default_rotation
    fa.goto_pose(pre_pour_pose)
    print('Moved to pre-pour pose')

    print('Press any key to close gripper')
    keyboard.read_key()
    fa.goto_gripper(0.002, grasp=True, force=25)
    print('Gripped bottle')

    time.sleep(1)

    force_norm_list = []
    for i in range(10):
        force = fa.get_ee_force_torque()[:3]
        force_norm_list.append(np.linalg.norm(force))
        time.sleep(0.2)
    # force_norm_list = np.array(force_norm_list)
    # avg_force_norm = np.mean(force_norm_list, axis=0)
    # print(fa.get_ee_force_torque()[:3])
    # print(f'Average force norm: {avg_force_norm}')

    print(f'Press any key to move to target pose: {TARGET_PITCH}')
    keyboard.read_key()

    # Select target pitch
    # pitch_idx = 1
    # pitch_list = list(np.linspace(0.0, -initial_pitch, 5))
    # target_pitch = pitch_list[pitch_idx]
    # target_pitch = -0.1

    final_pose = pre_pour_pose = RigidTransform(from_frame='franka_tool', to_frame='world')
    final_pose.translation = [0.450, 0.012, 0.350]
    target_pitch = TARGET_PITCH
    default_rotation = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
    additional_rotation = np.array([[np.cos(target_pitch), 0, np.sin(target_pitch)], [0, 1, 0], [-np.sin(target_pitch), 0, np.cos(target_pitch)]])
    final_rotation = default_rotation @ additional_rotation
    final_pose.rotation = final_rotation
    fa.goto_pose(final_pose)
    print(f'Moved to target pitch pose: {initial_pitch + target_pitch}')

    scale = WeighingScale()


    current_weight = scale.read_weight()
    while True:
        time.sleep(0.05)
        new_weight = scale.read_weight()
        print(f'Current weight: {new_weight} grams. Waiting for pouring to start...')
        if new_weight > current_weight + 0.1:
            print("Pouring started...")
            break

    tick = time.time()
    
    while(True):
        time.sleep(0.01)
        new_weight = scale.read_weight()
        if new_weight != -1:
            current_weight = new_weight
        
        print(f'Current weight: {current_weight} grams')
        if current_weight > 20:
            break
    tock = time.time()
    time_taken = tock - tick

    print(f'Time taken to pour: {time_taken} seconds')

    # while True:
    #     time.sleep(0.1)
    #     force_torque = fa.get_ee_force_torque()
    #     print(f'Force torque: {force_torque}')

if __name__ == '__main__':
    main()