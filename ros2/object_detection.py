import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import torch

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')

        # Initialize CV Bridge
        self.bridge = CvBridge()

        # Subscribe to the camera image topic
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # Load the YOLOv5 model
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s')  # Use the small YOLOv5 model

        self.get_logger().info('Object Detection Node initialized.')

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Perform object detection
            results = self.model(cv_image)

            # Render detections on the image
            detections = results.pandas().xyxy[0]  # Get detection data as pandas DataFrame

            for _, detection in detections.iterrows():
                x1, y1, x2, y2, conf, cls, label = (
                    int(detection['xmin']),
                    int(detection['ymin']),
                    int(detection['xmax']),
                    int(detection['ymax']),
                    detection['confidence'],
                    int(detection['class']),
                    detection['name']
                )
                # Draw a rectangle and label on the image
                cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(cv_image, f'{label} ({conf:.2f})', (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # Show the image with detections
            cv2.imshow('Object Detection', cv_image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')


def main(args=None):
    rclpy.init(args=args)

    node = ObjectDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Object Detection Node.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
