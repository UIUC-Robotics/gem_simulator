#!/usr/bin/env python3
"""
Condenses /collision (ros_gz_interfaces/Contacts, one big message per physics
update) into one small JSON event on /collision_event each time the car newly
touches an obstacle.
"""

import json
import math

import rclpy
from rclpy.node import Node
from ros_gz_interfaces.msg import Contacts
from std_msgs.msg import String


class CollisionEvents(Node):
    def __init__(self):
        super().__init__('collision_events')
        self.declare_parameter('car_prefix', 'gem::')
        # Contact must be absent this long (sim seconds) before the same obstacle counts as a new hit
        self.declare_parameter('rehit_gap', 0.5)
        self.car_prefix = self.get_parameter('car_prefix').value
        self.rehit_gap = self.get_parameter('rehit_gap').value
        self.last_seen = {}
        self.pub = self.create_publisher(String, 'collision_event', 10)
        self.create_subscription(Contacts, 'collision', self.callback, 50)

    def callback(self, msg):
        now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        per_obstacle = {}
        for c in msg.contacts:
            if c.collision1.name.startswith(self.car_prefix):
                other = c.collision2.name
            else:
                other = c.collision1.name
            per_obstacle.setdefault(other, []).append(c)

        for other, contacts in per_obstacle.items():
            is_new = now - self.last_seen.get(other, -math.inf) > self.rehit_gap
            self.last_seen[other] = now
            if not is_new:
                continue
            points = [p for c in contacts for p in c.positions]
            forces = [
                math.sqrt(w.body_1_wrench.force.x ** 2 + w.body_1_wrench.force.y ** 2 + w.body_1_wrench.force.z ** 2)
                for c in contacts for w in c.wrenches
            ]
            event = {
                'time': round(now, 3),
                'obstacle': other.split('::')[0],
                'obstacle_collision': other,
                'num_points': len(points),
                'position': {
                    'x': sum(p.x for p in points) / len(points),
                    'y': sum(p.y for p in points) / len(points),
                    'z': sum(p.z for p in points) / len(points),
                } if points else None,
                'max_depth': max((d for c in contacts for d in c.depths), default=0.0),
                'max_force': max(forces, default=0.0),
            }
            self.pub.publish(String(data=json.dumps(event)))
            self.get_logger().info(f"Collision with {event['obstacle']}")


def main(args=None):
    rclpy.init(args=args)
    node = CollisionEvents()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
