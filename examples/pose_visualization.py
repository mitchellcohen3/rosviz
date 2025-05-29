import numpy as np  
import rospy
from rosviz.types import OdometryViz
from rosviz.utils import Exp_SO3

def main(n_timestamps):
    # Create an odometry visualization object
    odometry_viz = OdometryViz()

    # Update the odometry visualization with new poses
    position = np.array([0.0, 0.0, 0.0])
    attitude = Exp_SO3(np.array([0.0, 0.0, 0.0]))

    for i in range(n_timestamps):
        position += np.array([0.01, 0.01, 0.0])
        attitude = attitude @ Exp_SO3(np.array([0.0, 0.0, 0.01]))
        odometry_viz.update(attitude, position)
        rospy.sleep(0.01)
    
    # Spin to keep the node alive
    rospy.spin()


if __name__ == "__main__":
    rospy.init_node("pose_visualization_example")
    n_timestamps = 2000
    main(n_timestamps)