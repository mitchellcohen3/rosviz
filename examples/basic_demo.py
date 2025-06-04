"""An example of how to visualize poses, camera frames, and point clouds in ROS using rosviz."""

import numpy as np  
import rospy
from rosviz.types import OdometryViz, PointCloudViz, CameraViz
from rosviz.utils import Exp_SO3

def main(n_timestamps):
    # Create an odometry viz, point cloud viz, and camera viz object
    odometry_viz = OdometryViz(enable_path_viz=True)
    point_cloud_viz = PointCloudViz()
    camera_viz = CameraViz(enable_path_viz=False, r=1.0, g=0.1, b=0.1, scale=0.4)

    # Generate points along a circle in the x-y plane
    theta = np.linspace(0, 2 * np.pi, 100)
    point_cloud_list = [np.array([5.0 * np.cos(t), 5.0 * np.sin(t), 0.0]) for t in theta]

    # Specify the transformation from the camera frame to the odometry frame
    T_bc = np.eye(4)
    T_bc[:3, :3] = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])
    T_bc[:3, 3] = np.array([0.2, 0.0, 0.3])

    stamps = np.linspace(0, 2 * np.pi, n_timestamps)
    attitude = np.eye(3)
    for stamp in stamps:

        # Update the odometry pose
        position = np.array([3.0 * np.sin(stamp), 3.0 * np.cos(stamp), 0.5 * np.sin(stamp)])
        attitude = attitude @ Exp_SO3(np.array([0.001, 0.001, 0.01]))

        # Compute the camera pose
        T_ab = np.eye(4)
        T_ab[:3, :3] = attitude
        T_ab[:3, 3] = position
        T_ac = T_ab @ T_bc

        odometry_viz.update(attitude, position)
        point_cloud_viz.update(point_cloud_list)
        camera_viz.update(T_ac[:3, :3], T_ac[:3, 3])
        rospy.sleep(0.0005)
    

if __name__ == "__main__":
    rospy.init_node("pose_visualization_example")
    n_timestamps = 2000
    main(n_timestamps)