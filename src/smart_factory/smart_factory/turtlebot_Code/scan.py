import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import cv2
import math
import os
import time
class Joint1RotationNode(Node):
    def __init__(self):
        super().__init__('joint1_rotation_node')
        # joint_trajectory를 퍼블리시하는 퍼블리셔 생성
        self.publisher_ = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.sub_capture = self.create_subscription(String, 'capture', self.capture, 10)
        
        self.initial_pose_msg = JointTrajectory()
        self.initial_pose_msg.joint_names = ["joint1", "joint2", "joint3", "joint4"]
        initial_point = JointTrajectoryPoint()
        initial_point.positions = [0.0, -0.384, -0.262, 2.0]  # 초기 포즈: 모든 관절 0도
        initial_point.time_from_start.sec = 1  # 복귀 시간: 1초
        self.initial_pose_msg.points.append(initial_point)
        self.initial_pose_sent = False
        # 카메라 이미지를 저장하기 위한 타이머 설정
        #self.camera_timer = self.create_timer(1.0, self.save_camera_image)
        # 주기적으로 joint 명령을 퍼블리시하기 위한 타이머 설정
        #self.timer = self.create_timer(0.1, self.publish_trajectory)
        # joint1의 회전 각도 변수 초기화
        self.joint1_angle = -100.0  # 시작 각도 (도)
        self.increment = 1.0       # 한 단계당 증가량 (도)
        self.direction = 1         # 움직이는 방향 (1: 증가, -1: 감소)
        # joint2, joint3, joint4의 미리 정의된 위치 설정
        self.joint_configurations = [
            {"joint2": -17.2, "joint3": -12.6, "joint4": 120.9},
            {"joint2": -17.2, "joint3": -12.6, "joint4": 120.9},
        ]
        self.current_config = 0  # 첫 번째 설정부터 시작
        # 작업 완료 플래그
        self.task_completed = False
        self.image_count = 160
        self.camera = cv2.VideoCapture(0, cv2.CAP_V4L2)
        self.camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        
        self.image_save_path = "./purple"

     # 이미지 저장 경로 설정
        # self.camera = cv2.VideoCapture('/dev/video1')
        # self.image_save_path = "/home/rokey10"
        # os.makedirs(self.image_save_path, exist_ok=True)
    
    def capture(self,msg):
        for _ in range(20):
            self.publish_trajectory()
            time.sleep(1)
            self.save_camera_image()

    def publish_trajectory(self):
        if self.task_completed:
            # 작업 완료 시 초기 포즈 메시지 퍼블리시 (한 번만 전송)
            if not self.initial_pose_sent:
                self.publisher_.publish(self.initial_pose_msg)
                self.get_logger().info("초기 포즈로 복귀 중...")
                self.initial_pose_sent = True  # 플래그 설정
            return
        # JointTrajectory 메시지 생성
        msg = JointTrajectory()
        msg.joint_names = ["joint1", "joint2", "joint3", "joint4"]
        point = JointTrajectoryPoint()
        # joint1 각도 업데이트
        self.joint1_angle += self.increment * self.direction
        # # joint1 각도가 제한 범위를 넘어가면 방향 전환
        # if self.joint1_angle > 40.0:
        #     self.joint1_angle = 40.0
        #     self.direction = -1
        #     self.switch_configuration()
        # elif self.joint1_angle < -40.0:
        #     self.joint1_angle = -40.0
        #     self.direction = 1
        #     self.switch_configuration()
        # joint1 각도와 고정된 각도를 추가
        joint2 = self.joint_configurations[self.current_config]["joint2"]
        joint3 = self.joint_configurations[self.current_config]["joint3"]
        joint4 = self.joint_configurations[self.current_config]["joint4"]
        point.positions = [
            math.radians(self.joint1_angle),  # 각도를 라디안으로 변환
            math.radians(joint2),
            math.radians(joint3),
            math.radians(joint4)
        ]
        point.time_from_start.sec = 1  # 동작 시간: 1초
        # trajectory 메시지에 포인트 추가
        msg.points.append(point)
        # 메시지 퍼블리시
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published Joint Positions: joint1={self.joint1_angle:.2f}, "
                               f"joint2={joint2}, joint3={joint3}, joint4={joint4}")
        # 작업 종료 조건 확인
        if self.joint1_angle == -40.0 and self.current_config == 0 and self.direction == 1:
            self.task_completed = True
            self.get_logger().info("작업이 완료되었습니다.")
    def switch_configuration(self):
        # 두 가지 설정을 번갈아 전환
        self.current_config = (self.current_config + 1) % len(self.joint_configurations)
    def save_camera_image(self):
        if self.task_completed:
            return
        ret, frame = self.camera.read()
        
        # 파일 이름 생성 및 저장
        image_filename = os.path.join(self.image_save_path, f"image_{self.image_count}.jpg")
        cv2.imwrite(image_filename, frame)
        self.image_count += 1
        self.get_logger().info(f"이미지가 저장되었습니다: {image_filename}")
        
    def destroy_node(self):
        # 종료 시 카메라 리소스 해제
        self.camera.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = Joint1RotationNode()
    rclpy.spin_once(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()