#!/usr/bin/env python3
import math
import time
import json

import rclpy
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String


# --- Helper function to build a PoseStamped ---
def make_pose(x: float, y: float, yaw: float) -> PoseStamped:
    """
    Create a PoseStamped (position + orientation) in the 'map' frame.
    """
    ps = PoseStamped()
    ps.header.frame_id = 'map'  # Nav2 goals are usually in 'map' frame
    ps.pose.position.x = x
    ps.pose.position.y = y

    # Convert yaw (in radians) into quaternion (Z-only)
    half = yaw * 0.5
    ps.pose.orientation.z = math.sin(half)
    ps.pose.orientation.w = math.cos(half)
    return ps


def main():
    # 1) Initialise ROS 2 and create a node
    rclpy.init()
    node = rclpy.create_node('hs_waypoint_follower_nav2pose')

    # 2) Create an ActionClient for the NavigateToPose action
    client = ActionClient(node, NavigateToPose, 'navigate_to_pose')

    # Publisher: tell others when robot arrives; publishes to robot/arrived that robot is at location
    arrival_pub = node.create_publisher(String, 'robot/arrived', 10)

    # --- Function to send a goal and wait for result ---
    def send_and_wait(pose: PoseStamped) -> bool:
        node.get_logger().info('Waiting for Nav2 action server...')
        client.wait_for_server()

        # Update timestamp (required in headers)
        pose.header.stamp = node.get_clock().now().to_msg()

        # Wrap pose in a NavigateToPose goal message
        goal = NavigateToPose.Goal()
        goal.pose = pose

        # Simple feedback callback: prints distance left to target
        def feedback_cb(fb):
            try:
                dist = fb.feedback.distance_remaining
                node.get_logger().info(f'Distance remaining: {dist:.2f} m')
            except Exception:
                pass 

        # Send the goal
        send_future = client.send_goal_async(goal, feedback_callback=feedback_cb)
        rclpy.spin_until_future_complete(node, send_future)
        handle = send_future.result()
        if not handle or not handle.accepted:
            node.get_logger().error('Goal was rejected!')
            return False

        # Wait until navigation is finished
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future)
        result = result_future.result()
        if result is None:
            node.get_logger().error('No result returned.')
            return False

        node.get_logger().info('Goal reached successfully!')

        # after 'goal reached successfully', we publish "arrived"
        arrival_msg = String()
        arrival_msg.data = "arrived"
        arrival_pub.publish(arrival_msg)
        node.get_logger().info("📣 Published 'arrived' message for arm node.")

        return True

    # --- Define your goal locations ---
    locations = {
        'A': make_pose(3.05, -1.18, 0.00),
        'B': make_pose(3.13, -0.0893, 0.00),
        'C': make_pose(3.13, 1.17, -0.00143),
        'DROP': make_pose(0.5, 0.0, 0.0) #still need to define this; this is for the shelf area
    }

    # --- Subscriber callback; this function essentially deals with getting to the box locations---
    def goal_callback(msg: String):
        try:
            data = json.loads(msg.data)  # Parse JSON string
            location = data["box"]["location"] #get location
            node.get_logger().info(f"Received PLC location: {location}")

            if location in locations:
                pose = locations[location]
                send_and_wait(pose) #now we go the location
            else: #some error detection
                node.get_logger().warn(f"Unknown location '{location}' — skipping.")
        except Exception as e:
            node.get_logger().error(f"Failed to parse PLC message: {e}")

    # --- Subscribe directly to PLC topic, so we subscribe to the topic that gives us PLC info and then call goal_callback ---
    node.create_subscription(String, 'hmi/unified_status', goal_callback, 10)

    # this is for the second part, where we get notified that box is on the arm, and then go to dropoff point
    def arm_ready_callback(msg: String):
        data = msg.data.strip().lower()
        if data == "ready":
            node.get_logger().info("🤖 Arm says ready — moving to DROP point.")
            if 'DROP' in locations:
                send_and_wait(locations['DROP']) #going to dropoff point
            else:
                node.get_logger().warn("No DROP location defined.")

    # --- Subscribe to the arm topic ---
    node.create_subscription(String, 'whatever the topic is', arm_ready_callback, 10) #subscribing to get that info, idk what the topic is

    node.get_logger().info("Listening to PLC data and ready to move...")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
