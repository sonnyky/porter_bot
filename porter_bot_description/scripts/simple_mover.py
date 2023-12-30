import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class SimpleMover(Node):
    def __init__(self):
        super().__init__('simple_mover')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 0.5  # Adjust this value to change speed
        msg.angular.z = 0.0  # Adjust this value to change rotation
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    simple_mover = SimpleMover()
    rclpy.spin(simple_mover)
    simple_mover.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
