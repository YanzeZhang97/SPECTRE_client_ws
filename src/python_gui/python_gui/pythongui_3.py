import sys
import cv2
import time
from PyQt5.QtWidgets import QApplication, QMainWindow, QPushButton, QLabel, QVBoxLayout, QWidget
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QImage, QPixmap
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge


class ROS2Node(Node):
    def __init__(self):
        super().__init__('ros2_ui_node')
        self.publisher = self.create_publisher(String, 'ui_topic', 10)
        self.image_publisher = self.create_publisher(CompressedImage, 'video_frames', 10)
        self.subscription = self.create_subscription(
            String, 'ui_response_topic', self.listener_callback, 10)
        self.received_message = ""
        self.bridge = CvBridge()
        self.streaming = False  # Flag to track streaming state

    def send_message(self, message):
        msg = String()
        msg.data = message
        self.publisher.publish(msg)
        self.get_logger().info(f'Published message: "{message}"')

    def listener_callback(self, msg):
        self.received_message = msg.data
        self.get_logger().info(f'Received message: "{msg.data}"')

    def publish_frame(self, frame):
        # Convert the OpenCV image to a ROS CompressedImage
        if self.streaming:
            msg = self.bridge.cv2_to_compressed_imgmsg(frame, dst_format="jpeg")
            self.image_publisher.publish(msg)
            self.get_logger().info("Published a video frame")


class MainWindow(QMainWindow):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.initUI()

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_label)
        self.timer.start(100)

        # Initialize video capture and display
        self.capture = cv2.VideoCapture(0)  # Start the camera
        if not self.capture.isOpened():
            self.label.setText("Failed to open the camera.")
            return

        self.frame_timer = QTimer()
        self.frame_timer.timeout.connect(self.update_frame)
        self.frame_timer.start(30)  # Update every 30 ms (~33 fps)

        # Recording and streaming attributes
        self.is_recording = False
        self.is_streaming = False
        self.video_writer = None

    def initUI(self):
        self.setWindowTitle("ROS 2 PyQt UI with Live Camera Feed and Streaming")

        # Layout and widgets
        layout = QVBoxLayout()
        
        # Send message button
        self.button = QPushButton("Send Message")
        self.button.clicked.connect(self.on_button_clicked)
        layout.addWidget(self.button)
        
        # Record video button
        self.record_button = QPushButton("Record 3-Second Video")
        self.record_button.clicked.connect(self.start_recording)
        layout.addWidget(self.record_button)

        # Start/Stop streaming button
        self.stream_button = QPushButton("Start Streaming")
        self.stream_button.clicked.connect(self.toggle_streaming)
        layout.addWidget(self.stream_button)
        
        # Label to display received messages
        self.label = QLabel("Received message will appear here")
        layout.addWidget(self.label)

        # Video feed label
        self.video_label = QLabel("Live Camera Feed")
        self.video_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.video_label)
        
        # Set layout
        container = QWidget()
        container.setLayout(layout)
        self.setCentralWidget(container)

    def on_button_clicked(self):
        self.ros_node.send_message("Hello from PyQt!")

    def update_label(self):
        if self.ros_node.received_message:
            self.label.setText(f"Received: {self.ros_node.received_message}")

    def update_frame(self):
        # Capture frame-by-frame
        ret, frame = self.capture.read()
        if ret:
            # Convert the frame to RGB format
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb_frame.shape
            bytes_per_line = ch * w
            q_image = QImage(rgb_frame.data, w, h, bytes_per_line, QImage.Format_RGB888)
            self.video_label.setPixmap(QPixmap.fromImage(q_image))
            
            # Publish the frame if streaming is active
            if self.is_streaming:
                self.ros_node.publish_frame(frame)

    def start_recording(self):
        if not self.is_recording:
            self.is_recording = True
            self.record_button.setEnabled(False)
            self.label.setText("Recording video...")

            # Define the codec and create VideoWriter object
            fourcc = cv2.VideoWriter_fourcc(*'XVID')
            self.video_writer = cv2.VideoWriter('output.avi', fourcc, 20.0, (640, 480))

            # Record video for 3 seconds
            start_time = time.time()
            while time.time() - start_time < 3:
                ret, frame = self.capture.read()
                if ret:
                    # Write the frame to the video file
                    self.video_writer.write(frame)
                else:
                    break

            # Release resources
            self.video_writer.release()
            self.is_recording = False
            self.record_button.setEnabled(True)
            self.label.setText("Recording complete. Video saved as output.avi.")

    def toggle_streaming(self):
        # Toggle the streaming state
        self.is_streaming = not self.is_streaming
        self.ros_node.streaming = self.is_streaming
        if self.is_streaming:
            self.stream_button.setText("Stop Streaming")
            self.label.setText("Streaming video frames...")
        else:
            self.stream_button.setText("Start Streaming")
            self.label.setText("Streaming stopped.")

    def closeEvent(self, event):
        # Release the camera when closing the app
        self.capture.release()
        event.accept()


def main():
    rclpy.init()
    ros_node = ROS2Node()
    app = QApplication(sys.argv)
    window = MainWindow(ros_node)
    window.show()

    try:
        sys.exit(app.exec_())
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
