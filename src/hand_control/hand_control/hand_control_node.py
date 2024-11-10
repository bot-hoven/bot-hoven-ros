import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_srvs.srv import Empty

class HandControlNode(Node):
    def __init__(self):
        super().__init__('hand_control_node')
        
        # Publishers
        self.hand_position_cmd_pub = self.create_publisher(Float32, 'hand_position_cmd', 10)
        
        # Subscriber
        self.hand_position_state_sub = self.create_subscription(Float32, 'hand_position_state', self.hand_position_state_callback, 10)

        # Services
        self.set_hand_home_srv = self.create_service(Empty, 'set_hand_home', self.set_hand_home_callback)

    def hand_position_feedback_callback(self, msg):
        self.get_logger().info(f'Hand position feedback: {msg.data}')
        
    def set_hand_home_callback(self, request, response):
        self.get_logger().info('Setting hand to home position')
        return response
    
def main(args=None):
    rclpy.init(args=args)
    node = HandControlNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()