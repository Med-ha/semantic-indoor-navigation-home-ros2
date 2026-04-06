import os
import yaml

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped


class SaveNamedPlaceNode(Node):
    def __init__(self):
        super().__init__('save_named_place_node')

        self.declare_parameter(
            'places_file',
            os.path.expanduser('~/semantic_nav_ws/src/semantic_nav_bringup/config/named_places.yaml')
        )
        self.declare_parameter('place_name', 'kitchen')

        self.places_file = os.path.expanduser(self.get_parameter('places_file').value)
        self.place_name = str(self.get_parameter('place_name').value)

        self.clicked_sub = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.clicked_callback,
            10
        )

        self.get_logger().info(
            f"Ready to save clicked point as place '{self.place_name}'. Click once in RViz Publish Point."
        )

    def clicked_callback(self, msg: PointStamped):
        data = {'places': {}}

        if os.path.exists(self.places_file):
            with open(self.places_file, 'r') as f:
                loaded = yaml.safe_load(f)
                if isinstance(loaded, dict):
                    data = loaded

        if 'places' not in data:
            data['places'] = {}

        data['places'][self.place_name] = {
            'x': float(msg.point.x),
            'y': float(msg.point.y),
            'yaw': 0.0
        }

        with open(self.places_file, 'w') as f:
            yaml.safe_dump(data, f, sort_keys=True)

        self.get_logger().info(
            f"Saved place '{self.place_name}' at x={msg.point.x:.2f}, y={msg.point.y:.2f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = SaveNamedPlaceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
