import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool


class LightService(Node):
    def __init__(self):
        super().__init__('light_service_node')

        #create the service /switch_lights using SetBool
        self.srv = self.create_service(
            SetBool,
            '/switch_lights',
            self.switch_lights_callback
        )

        self.get_logger().info('Service /switch_lights is ready.')

    def switch_lights_callback(self, request, response):
        #check the boolean request
        if request.data:
            response.success = True
            response.message = 'ON'
            self.get_logger().info('Received request: turning lights ON')
        else:
            response.success = True
            response.message = 'OFF'
            self.get_logger().info('Received request: turning lights OFF')

        return response


def main(args=None):
    rclpy.init(args=args)

    node = LightService()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()