import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool

class FingerControlNode(Node):
    def __init__(self):
        super().__init__('finger_control_node')
        
        # Publishers
        self.finger_position_cmd_pub = self.create_publisher(Float32, 'finger_position_cmd', 10)
        self.finger_key_press_cmd_pub = self.create_publisher(Bool, 'finger_key_press_cmd', 10)
        
        # Subscribers
        self.finger_key_press_state_sub = self.create_subscription(Bool, 'finger_key_press_state', self.finger_key_press_state_callback, 10)
        
        # Parameters
        self.declare_parameter('finger_press_duration', 1.0)
        
    def finger_key_press_state_callback(self, msg):
        self.get_logger().info(f'Finger key press state: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = FingerControlNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()