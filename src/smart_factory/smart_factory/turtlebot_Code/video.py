import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from geometry_msgs.msg import Point
import cv2
from cv_bridge import CvBridge
from ultralytics import YOLO
import sys

class RobotNode(Node):
    def __init__(self):
        super().__init__('robotnode')
        self.cap = cv2.VideoCapture(1,cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        #self.cap.set(cv2.CAP_PROP_FPS, 25)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        if not self.cap.isOpened():
            self.get_logger().error("Failed to open camera!")
            return
        self.model = YOLO('best_red_blue.pt')
        self.bridge = CvBridge()
        self.pub_img = self.create_publisher(CompressedImage,'robot_img',10)
        self.pub_grip = self.create_publisher(Point,'/object_coordinates',10)
        self.pub_total_cnt = self.create_publisher(String,'count',10)
        self.sub_item = self.create_subscription(String,'Item',self.callback_item,10)
        self.sub_status = self.create_subscription(String,'status_convey',self.callback_status,10)
        self.sub_start = self.create_subscription(String,'start',self.start_callback,10)

        self.block_place = {}
        self.isNext = [False,False,False,False]
        self.get_red_count = 0
        self.get_blue_count = 0
        self.goal = 0
        self.total_count = 0

        self.start = False

        #self.cap_img()
    def callback_status(self, msg):
        self.get_logger().info(msg.data)
        if msg.data == 'short':
            self.move_arm()
        
    def callback_item(self, msg):
        jobs = msg.data.split()
        for job in jobs:
            job_count = job.split('*')
            if job_count[0] == 'red':
                self.get_red_count = int(job_count[1])
            elif job_count[0] == 'blue':
                self.get_blue_count = int(job_count[1])
            elif job_count[0] == 'goal':
                self.goal = int(job_count[1])
        self.total_count = self.get_red_count + self.get_blue_count
        
    def start_callback(self, msg):

        if self.start == False:
            self.start = True
            msg = String()
            msg.data = str(self.total_count)
            self.pub_total_cnt.publish(msg)
            self.cap_img()
        
    def cap_img(self):
        ret,frame = self.cap.read()
        result = self.model.predict(frame,conf=0.8)
        predict_frame = result[0].plot()
        self.pub_img.publish(self.bridge.cv2_to_compressed_imgmsg(predict_frame))
        boxes = result[0].boxes
        for box in boxes:
            xyxy = box.xyxy.cpu().detach().numpy().tolist()
            x = (xyxy[0][0]+xyxy[0][2])//2
            y = (xyxy[0][1]+xyxy[0][3])//2
            cls = box.cls.cpu().detach().numpy().tolist()[0]
            if x < 640 and y < 360:
                if cls == 0.0 and self.get_red_count > 0:
                    self.block_place['2'] = ['red']
                    self.get_red_count -= 1
                elif cls == 1.0 and self.get_blue_count > 0:
                    self.block_place['2'] = ['blue']
                    self.get_blue_count -= 1
            elif x > 640 and y < 360:
                if cls == 0.0 and self.get_red_count > 0:
                    self.block_place['1'] = ['red']
                    self.get_red_count -= 1
                elif cls == 1.0 and self.get_blue_count > 0:
                    self.block_place['1'] = ['blue']
                    self.get_blue_count -= 1
            elif x < 640 and y > 360:
                if cls == 0.0 and self.get_red_count > 0:
                    self.block_place['3'] = ['red']
                    self.get_red_count -= 1
                elif cls == 1.0 and self.get_blue_count > 0:
                    self.block_place['3'] = ['blue']
                    self.get_blue_count -= 1
            elif x > 640 and y > 360:
                if cls == 0.0 and self.get_red_count > 0:
                    self.block_place['4'] = ['red']
                    self.get_red_count -= 1
                elif cls == 1.0 and self.get_blue_count > 0:
                    self.block_place['4'] = ['blue']
                    self.get_blue_count -= 1
        
        if self.get_blue_count > 0 or self.get_red_count > 0:
            self.get_logger().error("박스가 부족합니다. 실행 불가")
            sys.exit()
        else:
            self.move_arm()
        
        
    def move_arm(self):
        msg = Point()
        for key,value in self.block_place.items():
            self.get_logger().info(key)
            if key == '1' and self.isNext[0] == False:
                msg.x = 7.0
                msg.y = 1.0
                msg.z = 0.0
                #msg.x = 190.0
                #msg.y = -60.0
                #msg.z = -30.0
                self.isNext[0] = True
                self.pub_grip.publish(msg)
                break
            elif key == '2' and self.isNext[1] == False:
                msg.x = 7.0
                msg.y = -1.0
                msg.z = 0.0
                #msg.x = 190.0
                #msg.y = 60.0
                #msg.z = -30.0
                self.isNext[1] = True
                self.pub_grip.publish(msg)
                break
            elif key == '3' and self.isNext[2] == False:
                msg.x = 5.0
                msg.y = -1.0
                msg.z = 0.0
                #msg.x = 140.0
                #msg.y = 60.0
                #msg.z = -30.0
                self.isNext[2] = True
                self.pub_grip.publish(msg)
                break
            elif key == '4'and self.isNext[3] == False:
                msg.x = 5.0
                msg.y = 1.0
                msg.z = 0.0
                #msg.x = 140.0
                #msg.y = -60.0
                #msg.z = -30.0
                self.isNext[3] = True  
                self.pub_grip.publish(msg)
                break
        
if __name__ == '__main__':
    rclpy.init()
    node = RobotNode()
    rclpy.spin(node)
    rclpy.shutdown()
    node.destroy_node()