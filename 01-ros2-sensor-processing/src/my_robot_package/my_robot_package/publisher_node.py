import os
import rclpy
from rclpy.node import Node
from my_robot_interfaces.msg import SensorData


class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher_node')

        #create publisher on topic /readings
        self.publisher_ = self.create_publisher(SensorData, '/readings', 10)

        #build a path to the sensor data file
        package_dir = os.path.dirname(os.path.dirname(__file__))
        file_path = os.path.join(package_dir, 'include', 'sensor_readings.txt')

        #read and save all measurements once at start 
        self.measurements = self.read_measurements(file_path)
        self.current_index = 0

        if not self.measurements:
            self.get_logger().error('No measurements found in sensor_readings.txt')
            return

        self.get_logger().info(f'Loaded {len(self.measurements)} measurements from file.')

        #publish one measurement per 1s (1Hz)
        self.timer = self.create_timer(1.0, self.publish_next_measurement)

    def read_measurements(self, file_path):
        measurements = []

        #check whether file exists
        if not os.path.exists(file_path):
            self.get_logger().error(f'File not found: {file_path}')
            return measurements

        try:
            #open the file 
            with open(file_path, 'r') as file:
                content = file.read()

            #split file into blocks using the separator
            blocks = content.split('*****')
            #removes empty lines and spaces
            for block in blocks:
                lines = []
                for line in block.splitlines():
                    line = line.strip()
                    if line:
                        lines.append(line)

                #skip empty blocks
                if len(lines) < 3:
                    continue
                #get the status, temperature and position values from the block and convert them to the correct types
                try:
                    status = lines[0]
                    temperature = float(lines[1])

                    position_values = [value.strip() for value in lines[2].split(',')]
                    position = [float(value) for value in position_values]

                    measurement = {
                        'status': status,
                        'temperature': temperature,
                        'position': position
                    }

                    measurements.append(measurement)

                except ValueError as e:
                    self.get_logger().warning(f'Skipping invalid block: {lines}. Error: {e}')

        except Exception as e:
            self.get_logger().error(f'Could not read file: {e}')

        return measurements

    def publish_next_measurement(self):
        #stop when all measurements are published
        if self.current_index >= len(self.measurements):
            self.get_logger().info('All measurements published')
            self.timer.cancel()
            return

        data = self.measurements[self.current_index]
        #create a SensorData message and fill it with the current measurement data
        msg = SensorData()
        msg.status = data['status']
        msg.temperature = data['temperature']
        msg.position = data['position']
        #send the message to the topic
        self.publisher_.publish(msg)

        self.get_logger().info(
            f'Published: status={msg.status}, temperature={msg.temperature}, position={msg.position}'
        )

        self.current_index += 1


def main(args=None):
    rclpy.init(args=args)

    node = PublisherNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()