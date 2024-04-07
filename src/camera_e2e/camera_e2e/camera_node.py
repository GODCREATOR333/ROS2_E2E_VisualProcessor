import rclpy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from rclpy.node import Node

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self._logger.info("Connected")
        self.byte_subscriber = self.create_subscription(
            Image,
            '/camera/image_raw_byte',  # Byte array topic
            self.byte_callback,
            10)
        self.byte_subscriber  # prevent unused variable warning

        self.base64_subscriber = self.create_subscription(
            String,
            '/camera/image_raw_base64',  # Base64 URL topic
            self.base64_callback,
            10)
        self.base64_subscriber  # prevent unused variable warning

    def byte_callback(self, msg):
        # Process byte array image data
        self.get_logger().info('Received byte array image data: %s' % msg.data)

    def base64_callback(self, msg):
        # Process base64 URL image data
        self.get_logger().info('Received base64 URL image data: %s' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
