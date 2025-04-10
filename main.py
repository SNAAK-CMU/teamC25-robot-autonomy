#!/usr/bin/env python3

import rospy
import tf2_ros
import tf2_geometry_msgs  # This is used to handle tf2 transformations
from geometry_msgs.msg import TransformStamped, PoseStamped
from frankapy import FrankaArm
import pour_beads
from traj_opt import solve_bead_pour
import numpy as np
from weighing_scale import WeighingScale
import utils
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

def main():
    fa = FrankaArm(init_node=False)
    scale = WeighingScale()

    target_weight, current_weight = utils.get_weights(scale)
    X, U, success, t_vec, dt = solve_bead_pour(start_mass=current_weight, end_mass=target_weight, verbose=True)

    if not success: 
        raise Exception("No Feasible trajectory found")

    # TODO: pickup cup and bring to pre-pour position
    utils.pour_beads(fa, Xm=X, Um=U, t_vec=t_vec, scale=scale, at_pre_pour=False, dt=dt, verbose=False)

    final_weight = scale.weight_averaged()
    print(f"Error = {target_weight - final_weight} g")

def get_image():
    image_data = rospy.wait_for_message("/camera/color/image_raw", Image)
    depth_image_data = rospy.wait_for_message("/camera/aligned_depth_to_color/image_raw", Image)
    bridge = CvBridge()
    rgb_image = bridge.imgmsg_to_cv2(image_data, "bgr8")
    depth_image = bridge.imgmsg_to_cv2(depth_image_data, "16UC1")
    return rgb_image, depth_image

def get_transform():
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    rospy.sleep(1)
    try:
        transform = tf_buffer.lookup_transform('panda_link0', 'camera_link', rospy.Time(0))

        T = utils.transform_to_matrix(transform)
        camera_info = rospy.wait_for_message("/camera/color/camera_info", CameraInfo)
        K = camera_info.K 
        K_matrix = np.array(K).reshape((3, 3))
        print(K_matrix)
        return T, K_matrix

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        #rospy.logerr("Transform error: %s", e)
        return None

def add_marker(x, y, z):
    # Create a Marker to show the XYZ location in RViz
    marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)
    
    marker = Marker()
    marker.header.frame_id = "panda_link0"  # Set the frame_id (in which coordinate frame the marker will be published)
    marker.header.stamp = rospy.Time.now()

    marker.ns = "xyz_marker"
    marker.id = 0  # Unique ID for the marker
    marker.type = Marker.SPHERE  # Marker type: SPHERE to represent a point
    marker.action = Marker.ADD

    # Set the position of the marker
    marker.pose.position = Point(x, y, z)
    marker.pose.orientation.w = 1.0  # No rotation

    # Set the scale (size of the sphere)
    marker.scale.x = 0.05  # Radius of the sphere
    marker.scale.y = 0.05
    marker.scale.z = 0.05

    # Set the color of the marker (RGB format, alpha = 1.0 for full opacity)
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 1.0
    rate = rospy.Rate(1)  # 1 Hz, adjust as needed
    # Publish the marker
    while not rospy.is_shutdown():
        marker.header.stamp = rospy.Time.now()
        marker_pub.publish(marker)
        rate.sleep()

if __name__ == "__main__":
    rospy.init_node('pour_beads', anonymous=True)
    
    # Call the function to get the required transform
    T, K = get_transform()

    # Get image data
    rgb, depth = get_image()

    # Get the pickup location
    X, Y, Z = utils.get_pickup_location(rgb, depth, T, K)
    print(f"X: {X}, Y: {Y}, Z: {Z}")

    # Add marker to RViz at the calculated location
    add_marker(X, Y, Z)

    # Run the main task (pouring beads)
    # main()
