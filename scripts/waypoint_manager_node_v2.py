#!/usr/bin/env python3

"""
Waypoint Manager Node (Updated: No Finish Flag)
author: Toshiki Nakamura
"""

from dataclasses import dataclass

import rospy
import yaml
from geometry_msgs.msg import (
    Point,
    PoseStamped,
    PoseWithCovarianceStamped,
    Quaternion,
    Vector3,
)
from std_msgs.msg import ColorRGBA
from std_srvs.srv import SetBool, SetBoolResponse
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker, MarkerArray


@dataclass(frozen=True)
class Params:
    """Data class for parameters"""

    frame_id: str = "map"
    waypoint_file: str = "waypoints.yaml"
    start: int = 0
    hz: int = 1
    width_ratio: float = 1.0
    is_visible_text: bool = True
    is_visible_edge: bool = True

    def print(self) -> None:
        """Print parameters"""
        rospy.loginfo(f"frame_id: {self.frame_id}")
        rospy.loginfo(f"waypoint_file: {self.waypoint_file}")
        rospy.loginfo(f"start: {self.start}")
        rospy.loginfo(f"hz: {self.hz}")
        rospy.loginfo(f"width_ratio: {self.width_ratio}")
        rospy.loginfo(f"is_visible_text: {self.is_visible_text}")
        rospy.loginfo(f"is_visible_edge: {self.is_visible_edge}")


class WaypointManager:
    """Class for managing waypoints"""

    def __init__(self) -> None:
        """Initialize waypoint manager"""

        rospy.init_node("waypoint_manager")
        self._params: Params = Params(
            frame_id=rospy.get_param("~frame_id", "map"),
            waypoint_file=rospy.get_param("~waypoint_file", "waypoints.yaml"),
            start=rospy.get_param("~start", 0),
            hz=rospy.get_param("~hz", 5),
            width_ratio=rospy.get_param("~width_ratio", 1.0),
            is_visible_text=rospy.get_param("~is_visible_text", True),
            is_visible_edge=rospy.get_param("~is_visible_edge", True),
        )

        self._waypoint_pub: rospy.Publisher = rospy.Publisher(
            "~waypoints", MarkerArray, queue_size=1, latch=True
        )
        self._goal_pose_pub: rospy.Publisher = rospy.Publisher(
            "~global_goal", PoseStamped, queue_size=1, latch=True
        )
        self._initialpose_pub: rospy.Publisher = rospy.Publisher(
            "/initialpose", PoseWithCovarianceStamped, queue_size=1, latch=True
        )

        rospy.loginfo(f"{rospy.get_name()} node has started...")
        rospy.loginfo("Parameters:")
        self._params.print()

        self._waypoints: list = self._load_waypoints(self._params.waypoint_file)
        self._update_count: int = self._params.start
        self._goal_pose: PoseStamped = PoseStamped()
        self._goal_pose.header.frame_id = self._params.frame_id

        # Initialize robot position if start index is valid
        if self._params.start < len(self._waypoints):
            rospy.sleep(1.0)
            initialpose: PoseWithCovarianceStamped = PoseWithCovarianceStamped()
            initialpose.header.frame_id = self._params.frame_id
            initialpose.header.stamp = rospy.Time.now()
            initialpose.pose.pose.position = Point(
                self._waypoints[self._params.start]["x"],
                self._waypoints[self._params.start]["y"],
                0.0,
            )
            initialpose.pose.pose.orientation = Quaternion(
                *quaternion_from_euler(0, 0, self._waypoints[self._params.start]["yaw"])
            )
            self._initialpose_pub.publish(initialpose)

    def _load_waypoints(self, file_path: str) -> list:
        """Load waypoints"""
        while not rospy.is_shutdown():
            try:
                with open(file_path, "r", encoding="utf-8") as file:
                    waypoints: list = yaml.safe_load(file)
                    if waypoints is not None:
                        return waypoints
            except Exception as e:
                rospy.logerr_throttle(5.0, e)
                rospy.sleep(1.0)

    def _update_goal_pose(self) -> bool:
        """Send the next goal pose sequentially"""
        if self._update_count >= len(self._waypoints):
            rospy.loginfo("All waypoints processed.")
            rospy.signal_shutdown("Finished processing all waypoints.")  # Trigger shutdown
            return False

        rospy.loginfo(f"Publishing goal for waypoint {self._update_count}")
        waypoint = self._waypoints[self._update_count]
        self._goal_pose.pose.position = Point(waypoint["x"], waypoint["y"], 0.0)
        self._goal_pose.pose.orientation = Quaternion(
            *quaternion_from_euler(0, 0, waypoint["yaw"])
        )
        self._goal_pose.header.stamp = rospy.Time.now()
        self._goal_pose_pub.publish(self._goal_pose)

        self._update_count += 1
        return True

    def process(self) -> None:
        """Main process loop"""
        r = rospy.Rate(self._params.hz)
        while not rospy.is_shutdown():
            self._waypoints = self._load_waypoints(self._params.waypoint_file)
            msg: MarkerArray = self._create_visualization(self._waypoints)
            self._waypoint_pub.publish(msg)

            # Publish the next goal
            self._update_goal_pose()
            r.sleep()

    def _create_visualization(self, waypoints) -> MarkerArray:
        """Create visualization"""
        msg: MarkerArray = MarkerArray()
        for waypoint in waypoints:
            arrow_marker: Marker = self._create_marker(
                id=waypoint["id"],
                type=Marker.ARROW,
                scale=Vector3(
                    self._params.width_ratio * 0.7,
                    self._params.width_ratio * 0.2,
                    self._params.width_ratio * 0.2,
                ),
                rgba=ColorRGBA(0.88, 0.0, 1.0, 1.0),
                yaw=waypoint["yaw"],
                point=Point(waypoint["x"], waypoint["y"], 0.0),
            )
            msg.markers.append(arrow_marker)

            if self._params.is_visible_text:
                text_marker: Marker = self._create_marker(
                    id=100 + waypoint["id"],
                    type=Marker.TEXT_VIEW_FACING,
                    scale=Vector3(
                        self._params.width_ratio,
                        self._params.width_ratio,
                        self._params.width_ratio,
                    ),
                    rgba=ColorRGBA(0.0, 0.0, 0.0, 1.0),
                    point=Point(waypoint["x"], waypoint["y"], self._params.width_ratio * 0.5),
                )
                text_marker.text = str(waypoint["id"])
                msg.markers.append(text_marker)

        if self._params.is_visible_edge:
            for i in range(len(waypoints) - 1):
                line_marker: Marker = self._create_marker(
                    id=200 + i,
                    type=Marker.LINE_STRIP,
                    scale=Vector3(self._params.width_ratio * 0.1, 0.0, 0.0),
                    rgba=ColorRGBA(0.0, 0.0, 1.0, 0.5),
                )
                line_marker.points.append(Point(waypoints[i]["x"], waypoints[i]["y"], 0))
                line_marker.points.append(Point(waypoints[i + 1]["x"], waypoints[i + 1]["y"], 0))
                msg.markers.append(line_marker)

        return msg

    def _create_marker(
        self,
        id: int,
        type: int,
        scale: Vector3,
        rgba: ColorRGBA,
        yaw: float = 0.0,
        point: Point = Point(),
    ) -> Marker:
        """Create a marker"""
        marker: Marker = Marker()
        marker.header.frame_id = self._params.frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = "waypoints"
        marker.id = id
        marker.type = type
        marker.action = Marker.ADD
        marker.pose.position = point
        marker.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, yaw))
        marker.scale = scale
        marker.color = rgba
        marker.lifetime = rospy.Duration(1 / self._params.hz)
        return marker


if __name__ == "__main__":
    try:
        node = WaypointManager()
        node.process()
    except rospy.ROSInterruptException:
        rospy.loginfo("Exception caught")
