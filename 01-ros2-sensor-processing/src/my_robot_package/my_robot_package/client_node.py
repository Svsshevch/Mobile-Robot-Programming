import sys
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool


class LightClient(Node):
    def __init__(self):
        super().__init__('light_client_node')

        #create a client for the /switch_lights service
        self.client = self.create_client(SetBool, '/switch_lights')

        self.get_logger().info('Client node started.')

    def send_request(self, command):
        #wait for the service to become available with a timeout of 5 seconds
        if not self.client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Service /switch_lights is not available.')
            return

        #create request object 
        request = SetBool.Request()
        request.data = command

        self.get_logger().info(f'Sending request: {request.data}')

        #call the service asynchronously and get a future object
        future = self.client.call_async(request)

        #wait until the result comes back
        rclpy.spin_until_future_complete(self, future)

        #check if the call succeeded
        if future.result() is not None:
            response = future.result()
            self.get_logger().info(
                f'Success: {response.success}, message: {response.message}'
            )
        else:
            self.get_logger().error('Service call failed.')


def main(args=None):
    rclpy.init(args=args)

    user_input = sys.argv[1].lower()

    if user_input == 'true':
        command = True
    elif user_input == 'false':
        command = False
    else:
        print('Invalid argument. Use true or false')
        return

    node = LightClient()
    node.send_request(command)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()