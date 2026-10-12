import math
from typing import TYPE_CHECKING

from geometry_msgs.msg import TwistStamped

from ..flight_state import FlightState
from .handler import FlightStateHandler

if TYPE_CHECKING:
    from ..flight_test_node import FlightTestNode


class FlyingToTargetHandler(FlightStateHandler):

    def handle(self, node: "FlightTestNode") -> None:

        if node.odometry is None:
            return
        
        current_x = node.odometry.pose.pose.position.x
        current_y = node.odometry.pose.pose.position.y

        if node.target_x is None:
            node.start_x = current_x
            node.start_y = current_y
            node.target_x = node.start_x - node.DELTA_X
            node.target_y = node.start_y - node.DELTA_Y
            node.get_logger().info(
                f"Start: ({node.start_x:.2f}, {node.start_y:.2f})"
            )
            node.get_logger().info(
                f"Target: ({node.target_x:.2f}, {node.target_y:.2f})"
            )

        error_x = node.target_x - current_x
        error_y = node.target_y - current_y
        distance = math.sqrt(error_x ** 2 + error_y ** 2)

        node.get_logger().info(
            f"Pos: ({current_x:.2f}, {current_y:.2f}); "
            f"dist = {distance:.2f} m"
        )

        if distance <= node.TARGET_TOLERANCE:
            stop = TwistStamped()
            stop.header.stamp = node.get_clock().now().to_msg()
            stop.header.frame_id = "base_link"
            node.cmd_vel_publisher.publish(stop)

            node.get_logger().info(
                f"Target reached. Error: {distance:.3f} m"
            )
            node.state = FlightState.LANDING
            return

        speed = min(node.MAX_SPEED, distance)

        q = node.odometry.pose.pose.orientation
        current_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y ** 2 + q.z ** 2),
        )

        target_yaw = math.atan2(error_y, error_x)
        yaw_error = target_yaw - current_yaw

        if yaw_error > math.pi:
            yaw_error -= 2.0 * math.pi
        elif yaw_error < -math.pi:
            yaw_error += 2.0 * math.pi

        command = TwistStamped()
        command.header.stamp = node.get_clock().now().to_msg()
        command.header.frame_id = "base_link"

        if abs(yaw_error) > node.YAW_TOLERANCE:
            command.twist.linear.x = 0.0
        else:
            command.twist.linear.x = speed

        command.twist.linear.y = 0.0
        command.twist.linear.z = 0.0
        command.twist.angular.x = 0.0
        command.twist.angular.y = 0.0

        if yaw_error > node.MAX_YAW_RATE:
            command.twist.angular.z = node.MAX_YAW_RATE
        elif yaw_error < -node.MAX_YAW_RATE:
            command.twist.angular.z = -node.MAX_YAW_RATE
        else:
            command.twist.angular.z = yaw_error

        node.cmd_vel_publisher.publish(command)