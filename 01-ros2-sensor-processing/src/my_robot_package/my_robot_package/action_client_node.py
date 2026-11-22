import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from my_robot_interfaces.action import CalibrateGPS


class GPSCalibrationActionClient(Node):
    def __init__(self):
        super().__init__('gps_calibration_action_client')
        self._action_client = ActionClient(
            self,
            CalibrateGPS,
            '/calibrate_gps'
        )

        self.get_logger().info('Action client started.')
    #sends a calibration request to the action server
    def send_goal(self):
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()
        #create and send the goal message
        goal_msg = CalibrateGPS.Goal()
        goal_msg.start = True

        self.get_logger().info('Sending calibration goal...')
        #send the goal asynchronously and register callbacks for response and feedback
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        #return a future that will be set when the server responds to the goal request
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        #get the goal handle from the future result
        goal_handle = future.result()
        #check if the goal was accepted by the server
        if not goal_handle.accepted:
            self.get_logger().error('Calibration goal was rejected.')
            return

        self.get_logger().info('Calibration goal accepted.')
        #if accepted, we can wait for the result asynchronously and register a callback for when the result is ready
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    #runs every time the server sends feedback about the progress of the calibration
    def feedback_callback(self, feedback_msg):
        progress = feedback_msg.feedback.progress
        self.get_logger().info(f'Feedback: {progress:.1f}%')
        
    #runs when the server sends the final result of the calibration process
    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Calibration success: {result.success}')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    node = GPSCalibrationActionClient()
    node.send_goal()
    rclpy.spin(node)


if __name__ == '__main__':
    main()