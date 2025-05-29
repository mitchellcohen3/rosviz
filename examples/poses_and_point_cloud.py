"""An example of how to visualize poses and point clouds in ROS using rosviz."""

import numpy as np  
import rospy
from rosviz.types import OdometryViz, PointCloudViz
from rosviz.utils import Exp_SO3

def main(n_timestamps):
    # Create an odometry visualization object and a point cloud visualization
    odometry_viz = OdometryViz(enable_path_viz=False)
    point_cloud_viz = PointCloudViz()

    # initial position and attitude 
    position = np.array([0.0, 0.0, 0.0])
    attitude = Exp_SO3(np.array([0.0, 0.0, 0.0]))

    # Generate a random point cloud 
    point_cloud = np.random.rand(100, 3) * 10.0
    point_cloud_list = [row.ravel() for row in point_cloud]

    for i in range(n_timestamps):
        position += np.array([0.01, 0.01, 0.01 * np.sin(i * 0.01)])
        attitude = attitude @ Exp_SO3(np.array([0.001, 0.001, 0.01]))
        odometry_viz.update(attitude, position)
        point_cloud_viz.update(point_cloud_list)
        rospy.sleep(0.01)
    

if __name__ == "__main__":
    rospy.init_node("pose_visualization_example")
    n_timestamps = 2000
    main(n_timestamps)