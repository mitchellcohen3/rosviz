"""A module containing useful visualization types for use with RViz."""

import typing
import numpy as np
import rospy
from geometry_msgs.msg import (
    Point,
    Pose,
    PoseStamped,
    Quaternion,
    Twist,
    Vector3,
)
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker
from sensor_msgs.msg import PointCloud
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped

from rosviz.utils import to_quat


class OdometryViz:
    def __init__(
        self,
        pub_name: str = "odometry_state",
        tf_pub_name: str = "/tf",
        queue_size: int = 50,
        frame_id="world",
        child_frame_id: str = "map",
        enable_path_viz: bool = False,
        max_poses_in_path: int = None,
        path_viz_pub_name: str = "path_viz",
    ):
        self.frame_id = frame_id
        self.child_frame_id = child_frame_id

        # Create odometry topic with the given frame_id and child_frame_id
        self.odom = Odometry()
        self.odom.header.frame_id = frame_id
        self.odom.child_frame_id = child_frame_id

        # Create publishers
        self.pub = rospy.Publisher(pub_name, Odometry, queue_size=queue_size)
        self.tf_pub = rospy.Publisher("/tf", TFMessage, queue_size=1)
        self.enable_path_viz = enable_path_viz

        if self.enable_path_viz:
            self.path_viz = PathViz(
                pub_name=path_viz_pub_name,
                max_poses=max_poses_in_path,
            )

    def update(self, attitude: np.ndarray, position: np.ndarray):
        """Updates the Odometry and publishes it over ROS.

        Parameters
        ----------
        attitude : np.ndarray
            Attitude of the robot, represented as a 3x3 direction cosine matrix (DCM)
        position : np.ndarray
            Position of the robot, represented as a 3x1 vector in the inertial frame
        """

        # Convert to quaternion
        quat = to_quat(attitude, order="xyzw")
        x, y, z = position.flatten()

        self.odom.pose.pose = Pose(
            Point(x, y, z),
            Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3]),
        )

        self.odom.twist.twist = Twist(
            Vector3(0, 0, 0),
            Vector3(0, 0, 0),
        )

        self.pub.publish(self.odom)

        # Publish the transform
        transform = TransformStamped()
        transform.header.frame_id = "world"
        transform.header.stamp = rospy.Time.now()
        transform.child_frame_id = "base_link"
        transform.transform.translation.x = position[0]
        transform.transform.translation.y = position[1]
        transform.transform.translation.z = position[2]
        transform.transform.rotation.w = quat[0]
        transform.transform.rotation.x = quat[1]
        transform.transform.rotation.y = quat[2]
        transform.transform.rotation.z = quat[3]

        tf_msg = TFMessage()
        tf_msg.transforms = [transform]
        self.tf_pub.publish(tf_msg)

        # Update path visualization if enabled
        if self.enable_path_viz:
            self.path_viz.update(attitude=attitude, position=position)


class EllipsoidViz:
    def __init__(
        self,
        color: str = "b",
        alpha: float = 0.3,
        id: int = 0,
        line_width=0.5,
        pub_name="ellipsoid",
    ):
        """Instantiate an Ellipsoid.

        Parameters
        ----------

        """
        self.color = ColorRGBA()
        if isinstance(color, np.ndarray):
            self.color.r = color[0]
            self.color.g = color[1]
            self.color.b = color[2]
            self.color.a = alpha
        else:
            if color == "b":
                self.color.r = 0.0
                self.color.g = 0.0
                self.color.b = 1.0
                self.color.a = alpha
            if color == "r":
                self.color.r = 1.0
                self.color.g = 0.0
                self.color.b = 0.0
                self.color.a = alpha
            if color == "g":
                self.color.r = 0.0
                self.color.g = 1.0
                self.color.b = 0.0
                self.color.a = alpha

        # Create marker
        marker = Marker()
        marker.ns = "Ellipsoid"
        marker.id = id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time.from_sec(1)
        marker.scale.x = line_width
        self.marker = marker

        self.pub = rospy.Publisher(pub_name, Marker, queue_size=50)

    def update(self, mean, scale, orientation):
        self.marker.pose.position.x = mean[0]
        self.marker.pose.position.y = mean[1]
        self.marker.pose.position.z = mean[2]
        self.marker.scale.x = scale[0]
        self.marker.scale.y = scale[1]
        self.marker.scale.z = scale[2]

        self.marker.color = self.color

        quat = utils.dcm_to_quaternion(orientation)
        self.marker.pose.orientation.x = quat[0]
        self.marker.pose.orientation.y = quat[1]
        self.marker.pose.orientation.z = quat[2]
        self.marker.pose.orientation.w = quat[3]
        self.pub.publish(self.marker)


class BoundingBox3DViz:
    def __init__(
        self,
        color: str = "b",
        alpha: float = 1.0,
        marker_id: int = 0,
        line_width=0.03,
        pub_name="bounding_box",
    ):
        """Instantiate a BoundingBox3D.

        Parameters
        ----------
        corners : np.ndarray
            3 x 8 matrix, where each column is a corner of the box
        color : str, optional
            Color of the bounding box, by default "b"
        alpha : float, optional
            Transparency, by default 1.0
        id : int, optional
            A unique id , by default 0
        line_width : float, optional
            Line width of visual, by default 0.5
        """

        self.color = ColorRGBA()
        if isinstance(color, tuple):
            self.color.r = color[0] / 256
            self.color.g = color[1] / 256
            self.color.b = color[2] / 256
            self.color.a = alpha
        if color == "b":
            self.color.r = 0.0
            self.color.g = 0.0
            self.color.b = 1.0
            self.color.a = alpha
        if color == "r":
            self.color.r = 1.0
            self.color.g = 0.0
            self.color.b = 0.0
            self.color.a = alpha
        if color == "g":
            self.color.r = 0.0
            self.color.g = 1.0
            self.color.b = 0.0
            self.color.a = alpha

        # Create marker
        marker = Marker()
        marker.ns = "3DBoundingBox"
        marker.id = marker_id
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time.from_sec(1)
        marker.scale.x = line_width
        self.marker = marker

        # Create a publisher for the 3d bounding box
        self.pub_name = pub_name
        self.pub = rospy.Publisher(pub_name, Marker, queue_size=50)

    def update(self, **kwargs):
        """Updates the bounding box and publishes on a given publisher.

        Box corner order is as follows:
                7 -------- 6
               /|         /|
              4 -------- 5 .
              | |        | |
              . 3 -------- 2
              |/         |/
              0 -------- 1,
        where the x direction points along the length (forward direction),
        y direction points along the width (left), and z-points up.

        Note that a reference point ``d`` on the object is chosen as the middle of the
        bottom plane, such that the 8 corners are given by

        r_od_d = np.array(
            [
                [-l / 2, -l / 2, l / 2, l / 2, -l / 2, -l / 2, l / 2, l / 2],
                [w / 2, -w / 2, -w / 2, w / 2, w / 2, -w / 2, -w / 2, w / 2],
                [0.0, 0.0, 0.0, 0.0, h, h, h, h],
            ]
        ),

        where o represents a corner point.

        Then, given the DCM C_ad, and the translation r_dw_a, each of the
        corner point resolved in the inertial frame are given by
            r_ow_a = C_ad * r_od_d + r_dw_a
        Parameters
        ----------
        orientation : np.ndarray
            3 x 3 DCM, C_ab
        position : np.ndarray
            3 x 1 position
        pub : rospy.Publisher
            publisher to publish this topic on
        """

        orientation = kwargs["orientation"]
        position = kwargs["position"].reshape((-1, 1))
        size = kwargs["size"]

        self.marker.points.clear()
        self.marker.colors.clear()

        # Generate the corners in the object frame
        l, w, h = size.ravel()
        corners_in_object = np.array(
            [
                [-l / 2, -l / 2, l / 2, l / 2, -l / 2, -l / 2, l / 2, l / 2],
                [w / 2, -w / 2, -w / 2, w / 2, w / 2, -w / 2, -w / 2, w / 2],
                [0.0, 0.0, 0.0, 0.0, h, h, h, h],
            ]
        )
        # If point z is the center point of the bounding box, and point c is a corner point,
        # the corners are r_pz_b
        # Position is r_zw_a, where w is a reference point.
        corners_in_inertial = np.zeros((3, 8))
        for i in range(corners_in_object.shape[1]):
            corners_in_inertial[:, [i]] = (
                orientation @ corners_in_object[:, [i]] + position
            )

        # This list specifies the connections between the corners that defines the bounding box
        connections = [
            [0, 1],
            [1, 2],
            [2, 3],
            [3, 0],  # Lower plane parallel to Z=0 plane
            [4, 5],
            [5, 6],
            [6, 7],
            [7, 4],  # Upper plane parallel to Z=0 plane
            [0, 4],
            [1, 5],
            [2, 6],
            [3, 7],  # Connections between upper and lower planes
        ]

        # Add points for each connection
        for connection in connections:
            points = corners_in_inertial[:, connection]
            p1 = points[:, 0].reshape((-1, 1))
            p2 = points[:, 1].reshape((-1, 1))

            p1 = utils.numpy_to_point(p1)
            p2 = utils.numpy_to_point(p2)

            self.marker.points.append(p1)
            self.marker.points.append(p2)
            self.marker.colors.append(self.color)
            self.marker.colors.append(self.color)

        self.pub.publish(self.marker)


class PathViz:
    def __init__(self, pub_name: str, max_poses: int = None):
        """Instantiate a PathViz object.

        This creates a publisher for a ROS Path message, which can be used to visualize
        paths in 3D space.

        Parameters
        ----------
        pub_name : str
            Name of the publisher for the path.
        max_poses : int, optional
            Maximum number of poses to store in the path, by default None.
            If None, the path can grow indefinitely.
        """

        self.path = Path()
        self.path.header.frame_id = "world"

        # Create a path publisher
        self.pub = rospy.Publisher(pub_name, Path, queue_size=10)
        self.max_poses = max_poses

    def update(self, attitude: np.ndarray, position: np.ndarray):
        """Updates the path object.

        Parameters
        ----------
        attitude : np.ndarray
            Orientation of the robot, represented as a 3x3 direction cosine matrix (DCM)
        position : np.ndarray
            Position of the robot, represented as a 3x1 vector in the inertial frame
        """

        # Create an odometry message and set the pose
        odom = Odometry()
        odom.header.frame_id = "world"
        odom.child_frame_id = "map"

        # Set the pose and twist
        x, y, z = position.flatten()
        quat = to_quat(attitude, order="xyzw")

        odom.pose.pose = Pose(
            Point(x, y, z),
            Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3]),
        )

        odom.twist.twist = Twist(
            Vector3(0, 0, 0),
            Vector3(0, 0, 0),
        )

        pose_stamped = PoseStamped()
        pose_stamped.header = odom.header
        pose_stamped.pose = odom.pose.pose

        # Add message to path
        self.path.poses.append(pose_stamped)

        if self.max_poses is not None:
            if len(self.path.poses) > self.max_poses:
                self.path.poses.pop(0)

        # Publish path
        self.pub.publish(self.path)


class CameraViz:
    """A class for visualizing a camera pose.

    Defines RVIz markers that represent a camera visual.
    See http://wiki.ros.org/rviz/DisplayTypes/Marker for more info about
    the marker type.

    Transferred from the C++ implementation of VINS-Fusion.
    """

    def __init__(
        self,
        r=1.0,
        g=0.0,
        b=0.0,
        a=1.0,
        scale=0.5,
        line_width=0.02,
        pub_name="camera_pose",
        clear_viz=True,
    ):
        """Instantiate a CameraViz object

        Parameters
        ----------
        r : float, optional
            _description_, by default 1.0
        g : float, optional
            _description_, by default 0.0
        b : float, optional
            _description_, by default 0.0
        a : float, optional
            _description_, by default 1.0
        scale : float, optional
            _description_, by default 0.5
        line_width : float, optional
            _description_, by default 0.02
        """
        # Set color
        self.color = ColorRGBA()
        self.color.r = r
        self.color.g = g
        self.color.b = b
        self.color.a = a

        # Main gemoetry properties - image corners
        self.imlt = np.array([-1.0, -0.5, 1.0]).reshape((-1, 1))
        self.imrt = np.array([1.0, -0.5, 1.0]).reshape((-1, 1))
        self.imlb = np.array([-1.0, 0.5, 1.0]).reshape((-1, 1))
        self.imrb = np.array([1.0, 0.5, 1.0]).reshape((-1, 1))

        # Top left indicator and optical center connector
        self.lt0 = np.array([-0.7, -0.5, 1.0]).reshape((-1, 1))
        self.lt1 = np.array([-0.7, -0.2, 1.0]).reshape((-1, 1))
        self.lt2 = np.array([-1.0, -0.2, 1.0]).reshape((-1, 1))
        self.oc = np.array([0.0, 0.0, 0.0]).reshape((-1, 1))

        # Scale and linewidth
        self.scale = scale
        self.line_width = line_width

        marker = Marker()
        marker.header.frame_id = "world"
        marker.ns = "CameraPoseVisualization"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = self.line_width

        # Position and orientation of marker
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0

        self.marker = marker

        self.pub = rospy.Publisher(pub_name, Marker, queue_size=50)
        self.clear_viz = clear_viz

    def update(self, **kwargs):
        """Updates the camera pose visual

        Parameters
        ----------
        C : np.ndarray
            Camera orientation
        r : np.ndarray
            Camera position
        """

        C = kwargs["orientation"]
        r = kwargs["position"].reshape((-1, 1))

        # Clear the points and color list
        if self.clear_viz:
            self.marker.points.clear()
            self.marker.colors.clear()

        # Compute all required quantities
        pt_lt = utils.numpy_to_point(C @ (self.scale * self.imlt) + r)
        pt_lb = utils.numpy_to_point(C @ (self.scale * self.imlb) + r)
        pt_rt = utils.numpy_to_point(C @ (self.scale * self.imrt) + r)
        pt_rb = utils.numpy_to_point(C @ (self.scale * self.imrb) + r)

        pt_lt0 = utils.numpy_to_point(C @ (self.scale * self.lt0) + r)
        pt_lt1 = utils.numpy_to_point(C @ (self.scale * self.lt1) + r)
        pt_lt2 = utils.numpy_to_point(C @ (self.scale * self.lt2) + r)
        pt_oc = utils.numpy_to_point(C @ (self.scale * self.oc) + r)

        # Image boundaries
        self.marker.points.append(pt_lt)
        self.marker.points.append(pt_lb)
        self.marker.colors.append(self.color)
        self.marker.colors.append(self.color)

        self.marker.points.append(pt_lb)
        self.marker.points.append(pt_rb)
        self.marker.colors.append(self.color)
        self.marker.colors.append(self.color)

        self.marker.points.append(pt_rb)
        self.marker.points.append(pt_rt)
        self.marker.colors.append(self.color)
        self.marker.colors.append(self.color)

        self.marker.points.append(pt_rt)
        self.marker.points.append(pt_lt)
        self.marker.colors.append(self.color)
        self.marker.colors.append(self.color)

        # Top-left indicator
        self.marker.points.append(pt_lt0)
        self.marker.points.append(pt_lt1)
        self.marker.colors.append(self.color)
        self.marker.colors.append(self.color)

        self.marker.points.append(pt_lt1)
        self.marker.points.append(pt_lt2)
        self.marker.colors.append(self.color)
        self.marker.colors.append(self.color)

        # Optical center Connector
        self.marker.points.append(pt_lt)
        self.marker.points.append(pt_oc)
        self.marker.colors.append(self.color)
        self.marker.colors.append(self.color)

        self.marker.points.append(pt_lb)
        self.marker.points.append(pt_oc)
        self.marker.colors.append(self.color)
        self.marker.colors.append(self.color)

        self.marker.points.append(pt_rt)
        self.marker.points.append(pt_oc)
        self.marker.colors.append(self.color)
        self.marker.colors.append(self.color)

        self.marker.points.append(pt_rb)
        self.marker.points.append(pt_oc)
        self.marker.colors.append(self.color)
        self.marker.colors.append(self.color)

        self.pub.publish(self.marker)


class PointCloudViz:
    def __init__(self, pub_name: str = "point_cloud"):
        self.pub = rospy.Publisher(pub_name, PointCloud, queue_size=50)

        self.point_cloud = PointCloud()
        self.point_cloud.header.frame_id = "world"

    def update(self, positions:typing.List[np.ndarray]):
        """Updates the point cloud.

        Parameters
        ----------
        positions : List[np.ndarray]
            List of point positions, resolved in the inertial frame.
        """

        if not isinstance(positions, list):
            positions = [positions]

        # Update the positions
        self.point_cloud.points.clear()
        for pos in positions:
            x, y, z = pos.flatten()
            self.point_cloud.points.append(Point(x, y, z))
        self.pub.publish(self.point_cloud)


class LineViz:
    def __init__(
        self,
        pub_name,
        r=1.0,
        g=0.0,
        b=0.0,
        a=1.0,
        line_width=0.04,
    ):
        # Set color
        self.color = ColorRGBA()
        self.color.r = r
        self.color.g = g
        self.color.b = b
        self.color.a = a

        # Create marker publisher
        self.pub = rospy.Publisher(pub_name, Marker, queue_size=50)

        # Create marker
        marker = Marker()
        marker.header.frame_id = "world"
        marker.ns = "LineViz"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = line_width

        # Position and orientation of marker
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0

        self.marker = marker

    def update(self, **kwargs):
        """Updates the line segment."""

        positions = kwargs["positions"]

        # Clear the points and color list
        self.marker.points.clear()
        self.marker.colors.clear()

        # Extract two positions and draw line
        r1 = utils.numpy_to_point(positions[0])
        r2 = utils.numpy_to_point(positions[1])

        self.marker.points.append(r1)
        self.marker.points.append(r2)
        self.marker.colors.append(self.color)
        self.marker.colors.append(self.color)

        self.pub.publish(self.marker)
