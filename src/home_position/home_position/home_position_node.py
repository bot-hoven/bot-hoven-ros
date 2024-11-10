import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import Empty

class HomePositionNode(Node):
    def __init__(self):
        super().__init__('home_position_node')
        
        # Publishers
        self.home_status_pub = self.create_publisher(Bool, 'home_status', 10)
        
        # Services
        self.calibrate_home_srv = self.create_service(Empty, 'calibrate_home', self.calibrate_home_callback)
        
    def calibrate_home_callback(self, request, response):
        self.get_logger().info('Calibrating home position')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = HomePositionNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()