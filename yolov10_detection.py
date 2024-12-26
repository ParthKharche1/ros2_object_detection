import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO

class YoloDetectionNode(Node):
    def __init__(self):
        super().__init__('yolo_detection_node')
        self.publisher_detections = self.create_publisher(String, 'yolo_detections', 10)
        self.publisher_frames = self.create_publisher(Image, 'yolo_frames', 10)
        self.bridge = CvBridge()

        # Load YOLO model
        self.model = YOLO('yolov10n.pt')
        self.classes = self.model.names
        np.random.seed(43210)
        self.colors = np.random.uniform(0, 255, size=(len(self.classes), 3))

        # Initialize video capture
        self.cap = cv2.VideoCapture(0)

        # Create a timer to process frames
        self.timer = self.create_timer(0.1, self.process_frame)

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning("Failed to capture frame from camera")
            return

        frame = cv2.flip(frame, 1)
        results = self.model(frame)

        detections = []
        for info in results:
            parameters = info.boxes
            for box in parameters:
                x1, y1, x2, y2 = box.xyxy[0].numpy().astype('int')
                confidence = box.conf[0].numpy().astype('float')
                class_detected_number = int(box.cls[0])
                class_detected_name = self.classes[class_detected_number]
                predicted_text = f"{class_detected_name}:{100 * confidence:.2f}%"
                detections.append(predicted_text)
                color = self.colors[class_detected_number]
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 3)
                cv2.putText(frame, predicted_text, 
                            (x1, y1 - 15 if y1 > 30 else y1 + 15), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        # Publish detections
        if detections:
            self.publisher_detections.publish(String(data=', '.join(detections)))

        # Publish annotated frame
        annotated_frame_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.publisher_frames.publish(annotated_frame_msg)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
