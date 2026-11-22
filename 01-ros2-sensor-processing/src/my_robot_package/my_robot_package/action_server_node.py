import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from my_robot_interfaces.action import CalibrateGPS


class GPSCalibrationActionServer(Node):
    def __init__(self):
        super().__init__('gps_calibration_action_server')

        self._action_server = ActionServer(
            self,
            CalibrateGPS,
            '/calibrate_gps',
            self.execute_callback
        )

        self.get_logger().info('Action server /calibrate_gps is ready.')

    def execute_callback(self, goal_handle):
        self.get_logger().info('Calibration request received.')

        feedback_msg = CalibrateGPS.Feedback()

        total_steps = 50   #5 seconds*10 Hz = 50 updates

        #simulate calibration process with progress updates
        for i in range(total_steps + 1):
            progress = (i / total_steps) * 100.0
            feedback_msg.progress = progress
            #send feedback to the client
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f'Calibration progress: {progress:.1f}%')

            time.sleep(0.1)   # 10 Hz (every 0.1 s)

        goal_handle.succeed()

        result = CalibrateGPS.Result()
        result.success = True

        self.get_logger().info('Calibration finished successfully.')
        return result


def main(args=None):
    rclpy.init(args=args)

    node = GPSCalibrationActionServer()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()