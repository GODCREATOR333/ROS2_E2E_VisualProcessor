import rclpy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from rclpy.node import Node
from ultralytics import YOLO
from ultralytics.models.yolo.detect.predict import DetectionPredictor
import cv2
import numpy as np

model = YOLO("yolov8s.pt")


from torch import cuda

device = 'cuda' if cuda.is_available() else 'cpu'
print(f"Using device: {device}")
model = YOLO("yolov8s.pt").to(device)

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self._logger.info("Connected")
        self.model = YOLO("yolov8s.pt")  # Initialize YOLO model
        self.byte_subscriber = self.create_subscription(
            Image,
            '/camera/image_raw_byte',  # Byte array topic
            self.byte_callback,
            10)
        self.byte_subscriber  # prevent unused variable warning

        # self.base64_subscriber = self.create_subscription(
        #     String,
        #     '/camera/image_raw_base64',  # Base64 URL topic
        #     self.base64_callback,
        #     10)
        # self.base64_subscriber  # prevent unused variable warning

        
    def byte_callback(self, msg):
        # Process byte array image data
        #self.get_logger().info('Received byte array image data: %s' % msg.data)

        # Convert byte array image data to numpy array
        np_arr = np.frombuffer(msg.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)


        # # Display the image
        # cv2.imshow('Image', image_np)
        # cv2.waitKey(1)  # Wait for a short time to allow the image to be displayed

        # Perform object detection using YOLO model
        results = self.model.predict(image_np, show=True)
        print(results)

    #def base64_callback(self, msg):
        # Process base64 URL image data
        #self.get_logger().info('Received base64 URL image data: %s' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
