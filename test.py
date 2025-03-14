import numpy as np
from weighing_scale import WeighingScale
import rospy

from autolab_core import RigidTransform
from frankapy import FrankaArm, SensorDataMessageType
from frankapy import FrankaConstants as FC
from frankapy.proto_utils import sensor_proto2ros_msg, make_sensor_group_msg

from frankapy.proto import PosePositionSensorMessage, ShouldTerminateSensorMessage, CartesianImpedanceSensorMessage
from franka_interface_msgs.msg import SensorDataGroup

from frankapy.utils import min_jerk, min_jerk_weight

if __name__ == '__main__':
    fa = FrankaArm()
    scale = WeighingScale()
    target_weight = scale.read_weight_on_key()
    current_weight = 0
    prev_e = 0
    dt = 0.01
    Kp = 1
    Kd = 1
    duration = np.inf
    default_rotation = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])

    # define publisher
    rospy.loginfo('Initializing Sensor Publisher')
    pub = rospy.Publisher(FC.DEFAULT_SENSOR_PUBLISHER_TOPIC, SensorDataGroup, queue_size=1000)
    rate = rospy.Rate(1 / dt)

    # move to x, y, and z directly above the bin
    pre_pour_pose = RigidTransform(from_frame='franka_tool', to_frame='world') # TODO get transform to bottle and define it as tool frame
    pre_pour_pose.translation = [0.3261, 0.012, 0.3447]
    pre_pour_pose.rotation = default_rotation

    fa.goto_pose(pre_pour_pose)

    fa.goto_pose(pre_pour_pose, duration=duration, dynamic=True, buffer_time=1)
    init_time = rospy.Time.now().to_time()

    message_id = 1 

    # pouring loop
    while (current_weight < target_weight):
        current_weight = scale.read_weight()
        e = target_weight - current_weight
        e_dot = prev_e - e / dt # TODO improve accuracy of dt
        pitch = Kp * e + Kd * e_dot

        # because of direction, to pour pitch needs to be negative
        pitch = np.clip(-pitch, -np.pi/3, 0)
        rotation = default_rotation @ np.array([[np.cos(pitch), 0, np.sin(pitch)], [0, 1, 0], [-np.sin(pitch), 0, np.cos(pitch)]])
        
        pour_pose = RigidTransform(from_frame='franka_tool', to_frame='world')
        pour_pose.rotation = rotation
        pour_pose.translation = pre_pour_pose.translation

        timestamp = rospy.Time.now().to_time() - init_time
        traj_gen_proto_msg = PosePositionSensorMessage(
            id=message_id, timestamp=timestamp, 
            position=pour_pose.translation, quaternion=pour_pose.quaternion #TODO check if quaternion works
        )
        ros_msg = make_sensor_group_msg(
            trajectory_generator_sensor_msg=sensor_proto2ros_msg(
                traj_gen_proto_msg, SensorDataMessageType.POSE_POSITION))
        
        rospy.loginfo('Publishing: ID {}'.format(traj_gen_proto_msg.id))
        pub.publish(ros_msg)
        rate.sleep()
    
    # Stop the skill
    # Alternatively can call fa.stop_skill()
    term_proto_msg = ShouldTerminateSensorMessage(timestamp=rospy.Time.now().to_time() - init_time, should_terminate=True)
    ros_msg = make_sensor_group_msg(
        termination_handler_sensor_msg=sensor_proto2ros_msg(
            term_proto_msg, SensorDataMessageType.SHOULD_TERMINATE)
        )
    pub.publish(ros_msg)

    rospy.loginfo('Done')