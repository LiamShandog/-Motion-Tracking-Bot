import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class SpeakerNode(Node):
    def __init__(self):
        super().__init__('speaker_node')

        # subscribe to IR sensor topic
        self.subscription = self.create_subscription(
            Bool,
            'ir_sensor',
            self.motion_callback,
            10
        )

        # To prevent overlapping triggers
        self.is_handling_motion = False

        self.get_logger().info('Speaker Node has been started.')

    def motion_callback(self, msg):
        if msg.data:
            if not self.is_handling_motion:
                self.is_handling_motion = True
                self.get_logger().info('Motion detected! Activating speaker and servo.')

def main(args=None):
    rclpy.init(args=args)
    node = SpeakerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()