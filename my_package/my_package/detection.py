import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String
import time

class DetectionNode(Node):
    def __init__(self):
        super().__init__('detection')
        self.image_publisher = self.create_publisher(Image, 'camera_image', 10)
        self.arm_publisher = self.create_publisher(String, 'activation_signal', 10)
        self.object_publisher = self.create_publisher(String, 'object_info', 10)
        self.bridge = CvBridge()

        # Try to connect to the camera otherwise keep trying
        while True:
            try:
                self.capture = cv2.VideoCapture(0)  # Change to 1 or 2 for different cameras
                if not self.capture.isOpened():  # Check if the capture object was initialized successfully
                    raise Exception("Failed to open camera...")  # Raise an exception if the camera couldn't be opened
                break
            except Exception as e:
                self.get_logger().info(f'{e}')
                # If an exception occurs while trying to connect, wait for a short time and try again
                self.get_logger().info('Retrying in 10 seconds...')
                time.sleep(10)  # Import time module if not already imported
        
        # Camera settings
        self.capture.set(cv2.CAP_PROP_BUFFERSIZE, 1) # 1 frame buffer
        self.frameCount = 0

        # File locations for yolov3 models IMPORTANT: MUST BE ABSOLUTE PATH
        weights_location = "/home/jmazon/ros2_ws/install/my_package/lib/python3.8/site-packages/my_package/models/yolov3-tiny.weights"
        cfg_location = "/home/jmazon/ros2_ws/install/my_package/lib/python3.8/site-packages/my_package/models/yolov3-tiny.cfg"
        coco_location= "/home/jmazon/ros2_ws/install/my_package/lib/python3.8/site-packages/my_package/models/coco.names"

        # Load YOLO model 
        self.net = cv2.dnn.readNet( weights_location, cfg_location)

        # Load COCO names 
        self.classes = []
        with open(coco_location, "r") as f:
            self.classes = [line.strip() for line in f]

        # Set the target classes
        self.target_classes = ['bottle', 'apple', 'cup', 'orange', 'banana']

        # Keep track of frames
        self.frameCount = 0

        # Create a timer that calls the publish_frame method every 1 second
        self.timer = self.create_timer(1, self.process_frame) # FPS of the camera

    def process_frame(self):
        ret, frame = self.capture.read()
        if ret:

            # Increment the frame count
            self.frameCount += 1

            # Get the height and width of the frame
            height, width, _ = frame.shape

            # Preprocess the frame for YOLO
            blob = cv2.dnn.blobFromImage(frame, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
            self.net.setInput(blob)
            outs = self.net.forward(self.net.getUnconnectedOutLayersNames())

            # Flags
            found_target = False 

            # Process each detection
            for out in outs:
                if found_target:  # If target is already found, break out of the outer loop
                    break
                for detection in out:
                    scores = detection[5:]
                    class_id = np.argmax(scores)
                    confidence = scores[class_id]

                    if confidence > 0.1 and self.classes[class_id] in self.target_classes:
                        
                        # Object detected
                        object = self.classes[class_id]
                        self.get_logger().info(f"{object} detected with confidence {confidence}")

                        # Draw bounding box
                        center_x = int(detection[0] * width)
                        center_y = int(detection[1] * height)
                        w = int(detection[2] * width)
                        h = int(detection[3] * height)

                        # Get x and y coordinates
                        x = int(center_x - w / 2)
                        y = int(center_y - h / 2)

                        # Calculate the center of the bounding box
                        center_x = x + w // 2
                        center_y = y + h // 2

                        # Draw rectangle around object
                        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

                        # Adjust coordinates to have (0,0) at the center of the frame
                        xcoordinate = center_x - (width // 2)
                        ycoordinate = (height // 2) - center_y
                        objectCoordinates = {'x': xcoordinate, 'y': ycoordinate}
                        objectCoordinatesString = f"x: {objectCoordinates['x']}, y: {objectCoordinates['y']}"
            
                        # Publish the object info ex: "apple x: 100, y: -30"
                        info = String()
                        info.data = f"{object} " + objectCoordinatesString
                        self.object_publisher.publish(info)

                        # Define the font scale and thickness
                        font_scale = 1
                        thickness = 2

                        # Calculate text size to determine text width and height
                        text_width, text_height = cv2.getTextSize(objectCoordinatesString, cv2.FONT_HERSHEY_SIMPLEX, font_scale, thickness)[0]

                        # Calculate the coordinates for placing the text centered and at the bottom
                        text_x = (width - text_width) // 2
                        text_y = height - 10  # You can adjust the value according to your preference

                        # Place coordinates on the frame
                        cv2.putText(frame, objectCoordinatesString, (text_x, text_y), cv2.FONT_HERSHEY_SIMPLEX, font_scale , (0, 255, 0), thickness)

                        # get rid of the 1 frame buffer
                        _, temp = self.capture.read()

                        # Set flag to True since target is found
                        found_target = True

                        break  # Break out of the inner loop

            # Publish the frame to the camera_image topic for viewing
            frame_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.image_publisher.publish(frame_msg)
            self.get_logger().info(f'Publishing: Frame #{self.frameCount}')

def main(args=None):
    rclpy.init(args=args)
    detection_node = DetectionNode()  
    rclpy.spin(detection_node)
    detection_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
