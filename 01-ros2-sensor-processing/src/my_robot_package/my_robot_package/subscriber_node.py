import rclpy
from rclpy.node import Node
import math
from my_robot_interfaces.msg import SensorData


class SensorSubscriber(Node):
    def __init__(self):
        super().__init__('sensor_subscriber')

        #create a subscriber to the /readings
        self.subscription = self.create_subscription(
            SensorData,              #message type
            '/readings',             #topic name
            self.listener_callback,  #callback function
            10                       #queue size
        )

        #store previous position to compute traveled distance
        self.previous_position = None

        #store total running distance
        self.total_distance = 0.0

        self.get_logger().info('Subscriber node started and waiting for data...')

    def listener_callback(self, msg):
        self.get_logger().info(f'Status: {msg.status}')

        #print a warning if temperature is >30 degrees
        if msg.temperature > 30.0:
            self.get_logger().warn(f'High temperature: {msg.temperature:.2f} °C')

        #get current position from the message
        current_position = msg.position

        #check that position contains exactly 3 values: x, y, z
        if len(current_position) != 3:
            self.get_logger().error('Invalid position! Expected [x, y, z]')
            return

        #compute distance only if there is a previous position
        if self.previous_position is not None:
            dx = current_position[0] - self.previous_position[0]
            dy = current_position[1] - self.previous_position[1]
            dz = current_position[2] - self.previous_position[2]

            step_distance = math.sqrt(dx**2 + dy**2 + dz**2)
            self.total_distance += step_distance

        #update previous position
        self.previous_position = current_position

        #print total running distance
        self.get_logger().info(f'Total distance: {self.total_distance:.3f} m')


def main(args=None):
    rclpy.init(args=args)

    node = SensorSubscriber()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()