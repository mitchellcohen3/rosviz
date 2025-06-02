"""An example of how to visualize camera poses in RViz."""

import numpy as np
import rospy
from rosviz.types import CameraViz
from rosviz.utils import Exp_SO3


def main(n_timestamps: int):

    # Create a camera visualization object
    # We can also optionally visualize the path of the camera
    camera_viz = CameraViz(
        enable_path_viz=True,
        path_viz_pub_name="camera_path",
        r=0.1,
        g=1.0,
        b=0.2,
        a=1.0
    )

    # Create a camera pose with z-axis pointing forward, y-axis pointing down, and x-axis to the right
    attitude = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])
    position = np.array([0.0, 0.0, 0.0])
    times = np.linspace(0, 10, n_timestamps)
    for i in range(n_timestamps):
        attitude = attitude
        position = np.array([1.0 * np.sin(times[i]), 1.0 * np.cos(times[i]), 0.1 * np.sin(times[i])])
        camera_viz.update(attitude, position)
        rospy.sleep(0.01)


if __name__ == "__main__":
    rospy.init_node("camera_visualization_example")
    n_timestmaps = 2000
    main(n_timestmaps)
