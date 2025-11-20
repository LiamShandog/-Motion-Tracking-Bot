# Python API for ROS 2
import rclpy
from rclpy.node import Node

# Import the Bool message type from std_msgs
from std_msgs.msg import Bool


# Declare IRSensorNode class, that inherits from Node
class IRSensorNode(Node):
    def __init__(self):
        # Inherit from Node class
        super().__init__('ir_sensor_node_test')

        self.publisher_ = self.create_publisher(Bool, 'ir_sensor', 10)  # Publisher for IR sensor data
        self.timer = self.create_timer(0.1, self.publish_data)  # Timer to periodically check sensor

    def publish_data(self):
        msg = Bool()
        msg.data = True
        self.publisher_.publish(msg) # Publish the message
        self.get_logger().info(f'Test motion: {msg.data}')  # Log the sensor state


def main(args=None):
    rclpy.init(args=args) # Initialize the rclpy library
    node = IRSensorNode() # Create an instance of the IRSensorNode
    try:
        rclpy.spin(node)      # Keep the node running
    except KeyboardInterrupt:
        pass    
    finally:
        rclpy.shutdown()     # Shutdown the rclpy library
    

if __name__ == '__main__': # Goes true if this script is run directly and not imported
    main()  # Call the main function