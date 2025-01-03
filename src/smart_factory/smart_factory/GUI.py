import cv2
from cv_bridge import CvBridge
from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel, QVBoxLayout, QHBoxLayout, QPushButton, QListWidget, QWidget, QLineEdit, QDialog, QFormLayout, QFrame
from PyQt5.QtGui import QPixmap, QImage
from PyQt5.QtCore import Qt, QTimer
import sys
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
import threading
import time

IsSuccess = False
IsOff = False

class GUINode(Node):
    def __init__(self):
        super().__init__('GUInode')
        self.cam_image = None
        self.robot_image = None

        self.sub_cam_image = self.create_subscription(CompressedImage, 'global_img', self.Image_callback, 10)
        self.sub_robot_image = self.create_subscription(CompressedImage, 'robot_img',self.robot_callback,10)

        self.pub_convey_status = self.create_publisher(String, 'status_convey', 10)
        self.pub_item = self.create_publisher(String, 'Item', 10)
        self.pub_capture = self.create_publisher(String,'capture',10)

        self.sub_sucess = self.create_subscription(String,'IsSuccess',self.success_callback,10)
        self.sub_pose = self.create_subscription(String, 're_pose', self.pose_callback,10)
        self.locate = ""

    def pose_callback(self, msg):
        self.get_logger().info(msg.data)
        self.locate = msg.data

    def Image_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        self.cam_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    def robot_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        self.robot_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    def success_callback(self, msg):
        global IsSuccess
        if msg.data == 'success':
            IsSuccess = True

class LoginDialog(QDialog):
    def __init__(self):
        super().__init__()
        self.init_ui()

    def init_ui(self):
        self.setWindowTitle("Login")
        self.setGeometry(150, 150, 300, 150)
        layout = QFormLayout()
        self.username_input = QLineEdit()
        self.password_input = QLineEdit()
        self.password_input.setEchoMode(QLineEdit.Password)
        self.login_button = QPushButton("Login")
        layout.addRow("Username:", self.username_input)
        layout.addRow("Password:", self.password_input)
        layout.addWidget(self.login_button)
        self.setLayout(layout)
        self.login_button.clicked.connect(self.handle_login)

    def handle_login(self):
        # Placeholder login logic
        if self.username_input.text() == "" and self.password_input.text() == "":
            self.accept()
        else:
            self.username_input.clear()
            self.password_input.clear()

class MainUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.init_ui()
        self.update_status('#87CEEB')
        self.node = None
        threading.Thread(target=self.global_cam_init, daemon=True).start()
        time.sleep(1)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_image)
        self.timer_robot = QTimer(self)
        self.timer_robot.timeout.connect(self.update_robot_image)
        self.timer.start(50)
        self.timer_robot.start(3000)


    def global_cam_init(self):
        rclpy.init()
        self.node = GUINode()
        rclpy.spin(self.node)
        rclpy.shutdown()
        self.node.destroy_node()

    def init_ui(self):
        self.setWindowTitle("ROS2 Image Viewer and Control")
        self.setGeometry(100, 100, 900, 700)
        
        # Set background color to white
        self.setStyleSheet("background-color: white;")

        # Main layout
        main_widget = QWidget()
        main_layout = QVBoxLayout()
        main_widget.setLayout(main_layout)
        self.setCentralWidget(main_widget)

        # Top Layout for images
        top_layout = QHBoxLayout()
        top_frame = QFrame()
        top_frame.setLayout(top_layout)
        top_frame.setStyleSheet("border: 1px solid #ccc; padding: 10px; background-color: white;")
        
        # Left image display
        self.left_image_label = QLabel("Left Image")
        self.left_image_label.setFixedSize(640, 360)
        self.left_image_label.setAlignment(Qt.AlignCenter)
        self.left_image_label.setStyleSheet("border: 1px solid black; background-color: #F0F0F0;")
        top_layout.addWidget(self.left_image_label)
        
        # Right image display
        self.right_image_label = QLabel("Right Image")
        self.right_image_label.setFixedSize(640, 360)
        self.right_image_label.setAlignment(Qt.AlignCenter)
        self.right_image_label.setStyleSheet("border: 1px solid black; background-color: #F0F0F0;")
        top_layout.addWidget(self.right_image_label)
        
        main_layout.addWidget(top_frame)

        # Bottom Layout for controls
        bottom_layout = QHBoxLayout()
        bottom_frame = QFrame()
        bottom_frame.setLayout(bottom_layout)
        bottom_frame.setStyleSheet("border: 1px solid #ccc; padding: 10px; background-color: white;")
        
        # Left list controls
        list_layout = QVBoxLayout()
        self.list1 = QListWidget()
        self.list1.setFixedHeight(100)
        self.list1.addItems(["red*2 blue*1 goal*1", "red*1 blue*2 goal*2", "red*1 goal*3"])
        self.list1.setStyleSheet("border: 1px solid black; background-color: #FFFFFF;")
        self.list1.itemDoubleClicked.connect(self.clicked_item)
        self.info_label1 = QLabel("Info Label 1")
        self.info_label1.setFixedWidth(120)
        self.info_label1.setAlignment(Qt.AlignCenter)
        self.info_label2 = QLabel("Info Label 2")
        self.info_label3 = QLabel("Info Label 3")
        list_layout.addWidget(self.list1)
        list_layout.addWidget(self.info_label1)
        list_layout.addWidget(self.info_label2)
        list_layout.addWidget(self.info_label3)
        bottom_layout.addLayout(list_layout)

        # Right button controls with better styling
        button_layout = QVBoxLayout()
        self.start_collection_button = QPushButton("수집 시작")
        self.start_convey_button = QPushButton("Conveyor Belt 시작")
        self.stop_convey_button = QPushButton("Conveyor Belt 정지")
        
        self.start_collection_button.setStyleSheet("""
            background-color: #4CAF50; 
            color: white; 
            padding: 10px; 
            font-size: 16px; 
            border-radius: 5px;
        """)
        self.start_convey_button.setStyleSheet("""
            background-color: #2196F3; 
            color: white; 
            padding: 10px; 
            font-size: 16px; 
            border-radius: 5px;
        """)
        self.stop_convey_button.setStyleSheet("""
            background-color: #F44336; 
            color: white; 
            padding: 10px; 
            font-size: 16px; 
            border-radius: 5px;
        """)
        
        button_layout.addWidget(self.start_collection_button)
        button_layout.addWidget(self.start_convey_button)
        button_layout.addWidget(self.stop_convey_button)
        bottom_layout.addLayout(button_layout)
        main_layout.addWidget(bottom_frame)

        # Connect signals to slots
        self.start_collection_button.clicked.connect(self.start_collection)
        self.start_convey_button.clicked.connect(self.start_convey)
        self.stop_convey_button.clicked.connect(self.stop_convey)

    def clicked_item(self):
        print(self.list1.currentItem().text())
        msg = String()
        msg.data = self.list1.currentItem().text()
        self.node.pub_item.publish(msg)
        self.update_robot_image()


    def success_status(self):
        global IsSuccess, IsOff
        while True:
            if IsSuccess == True:
                IsSuccess = False
                self.update_status('#81C147')
                time.sleep(2)
                self.update_status('#87CEEB')
                break
            elif IsOff == True:
                IsOff = False
                break
        
    def update_status(self,color):
        if color == '#4CAF50':
            text = 'convey 작동 중'
            text_color = "white"
        elif color == '#87CEEB':
            text = 'convey 대기 중'
            text_color = "black"
        elif color == '#81C147':
            text = 'convey 작동 완료'
            text_color = "white"
        
        self.info_label1.setText(text)
        self.info_label1.setStyleSheet(f"""
        background-color: {color};
        color: {text_color};
        padding: 5px;
        border-radius: 5px;
        """)

    def update_image(self):
        if self.node.cam_image is not None:
            rgb_image = cv2.cvtColor(self.node.cam_image, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb_image.shape
            bytes_per_line = ch * w
            qt_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
            pixmap = QPixmap.fromImage(qt_image)
            self.left_image_label.setPixmap(pixmap)
            self.left_image_label.setScaledContents(True)

    def update_robot_image(self):
        if self.node.robot_image is not None:
            rgb_image = cv2.cvtColor(self.node.robot_image, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb_image.shape
            bytes_per_line = ch * w
            qt_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
            pixmap = QPixmap.fromImage(qt_image)
            self.right_image_label.setPixmap(pixmap)
            self.right_image_label.setScaledContents(True)
        self.info_label2.setText(self.node.locate) 


    def start_collection(self):
        print("수집 시작 버튼이 눌렸습니다.")
        msg = String()
        msg.data = 'start'
        self.node.pub_capture(msg)

    def start_convey(self):
        global IsOff
        print("Convey 시작 버튼이 눌렸습니다.")
        IsOff = False
        start_msg = String()
        start_msg.data = 'start'
        self.node.pub_convey_status.publish(start_msg)
        self.update_status('#4CAF50')
        threading.Thread(target=self.success_status,daemon=True).start()

    def stop_convey(self):
        global IsOff
        print("Convey 정지 버튼이 눌렸습니다.")
        IsOff = True
        stop_msg = String()
        stop_msg.data = 'stop'
        self.node.pub_convey_status.publish(stop_msg)
        self.update_status('#87CEEB')
        

if __name__ == "__main__":
    app = QApplication(sys.argv)
    login_dialog = LoginDialog()
    if login_dialog.exec_() == QDialog.Accepted:
        main_window = MainUI()
        main_window.show()
        sys.exit(app.exec_())
