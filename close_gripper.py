from autolab_core import RigidTransform
from frankapy import FrankaArm, SensorDataMessageType
from frankapy import FrankaConstants as FC
from frankapy.proto_utils import sensor_proto2ros_msg, make_sensor_group_msg
import numpy as np

fa = FrankaArm()

# fa.goto_gripper(0.002, grasp=True, force=25)

# default_rotation = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
# initial_pitch = 0.25 #0.25
# additional_rotation = np.array([[np.cos(initial_pitch), 0, np.sin(initial_pitch)], [0, 1, 0], [-np.sin(initial_pitch), 0, np.cos(initial_pitch)]])
# default_rotation = default_rotation @ additional_rotation
# pitch = initial_pitch


# # move to x, y, and z directly above the bin
# pre_pour_pose = RigidTransform(from_frame='franka_tool', to_frame='world') # TODO get transform to bottle and define it as tool frame
# # pre_pour_pose.translation = [0.3261, 0.012, 0.3947] # [0.3261, 0.012, 0.3447]
# pre_pour_pose.translation = [0.45, 0.012, 0.350] # [0.3261, 0.012, 0.3447]
# pre_pour_pose.rotation = default_rotation
# fa.reset_joints()


fa.open_gripper()
print('Moved to pre-pour pose')