#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, PositionConstraint, BoundingVolume
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Header

TARGET_X = 0.3
TARGET_Y = 0.0
TARGET_Z = 0.4
TARGET_QX = 0.0
TARGET_QY = 0.0
TARGET_QZ = 0.0
TARGET_QW = 1.0

PLANNING_GROUP = "arm"
TIP_LINK = "Link_5_1"
BASE_FRAME = "base_link"


def main():
    


if __name__ == "__main__":
    main()
