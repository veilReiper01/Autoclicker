
import numpy as np
import cv2
import tf
from geometry_msgs.msg import PoseStamped

rvec, tvec = cv2.some_function()

# we need a homogeneous matrix but OpenCV only gives us a 3x3 rotation matrix
rotation_matrix = np.array([[0, 0, 0, 0],
                            [0, 0, 0, 0],
                            [0, 0, 0, 0],
                            [0, 0, 0, 1]],
                            dtype=float)
rotation_matrix[:3, :3], _ = cv2.Rodrigues(rvec)

# convert the matrix to a quaternion
quaternion = tf.transformations.quaternion_from_matrix(rotation_matrix)

# To visualize in rviz, you can, for example, publish a PoseStamped message:
pose = PoseStamped()
pose.header.frame_id = "camera_frame"
pose.pose.position.x = tvec[0]
pose.pose.position.y = tvec[1]
pose.pose.position.z = tvec[2]
pose.pose.orientation.x = quaternion[0]
pose.pose.orientation.y = quaternion[1]
pose.pose.orientation.z = quaternion[2]
pose.pose.orientation.w = quaternion[3]

pose_publisher.publish(pose)
