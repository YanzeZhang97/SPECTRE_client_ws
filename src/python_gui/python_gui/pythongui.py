import sys
import cv2
import time
from PyQt5.QtWidgets import QApplication, QMainWindow, QPushButton, QLabel, QVBoxLayout, QWidget
from PyQt5.QtCore import QTimer, Qt
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ROS2Node(Node):
    def __init__(self):
        super().__init__('ros2_ui_node')
        self.publisher = self.create_publisher(String, 'ui_topic', 10)
        self.subscription = self.create_subscription(
            String, 'ui_response_topic', self.listener_callback, 10)
        self.received_message = ""

    def send_message(self, message):
        msg = String()
        msg.data = message
        self.publisher.publish(msg)
        self.get_logger().info(f'Published message: "{message}"')

    def listener_callback(self, msg):
        self.received_message = msg.data
        self.get_logger().info(f'Received message: "{msg.data}"')


class MainWindow(QMainWindow):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.initUI()

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_label)
        self.timer.start(100)

        # Initialize video capture attributes
        self.is_recording = False
        self.video_writer = None
        self.capture = None

    def initUI(self):
        self.setWindowTitle("ROS 2 PyQt UI with Video Recording")

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
        
        # Label to display received messages
        self.label = QLabel("Received message will appear here")
        layout.addWidget(self.label)
        
        # Set layout
        container = QWidget()
        container.setLayout(layout)
        self.setCentralWidget(container)

    def on_button_clicked(self):
        self.ros_node.send_message("Hello from PyQt!")

    def update_label(self):
        if self.ros_node.received_message:
            self.label.setText(f"Received: {self.ros_node.received_message}")

    def start_recording(self):
        if not self.is_recording:
            self.is_recording = True
            self.record_button.setEnabled(False)
            self.label.setText("Recording video...")

            # Initialize the camera and video writer
            self.capture = cv2.VideoCapture(0)
            if not self.capture.isOpened():
                self.label.setText("Failed to open the camera.")
                self.is_recording = False
                self.record_button.setEnabled(True)
                return

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
            self.capture.release()
            self.video_writer.release()
            self.is_recording = False
            self.record_button.setEnabled(True)
            self.label.setText("Recording complete. Video saved as output.avi.")


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
