import rclpy
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.action import ActionClient
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import String, Header
import math
import time


j1_z_offset = 77
r1 = 130
r2 = 124
r3 = 150
th1_offset = -math.atan2(0.024, 0.128)
th2_offset = -0.5 * math.pi - th1_offset
# Function to calculate joint angles
def solv2(r1, r2, r3):
    d1 = (r3**2 - r2**2 + r1**2) / (2 * r3)
    d2 = (r3**2 + r2**2 - r1**2) / (2 * r3)
    s1 = math.acos(d1 / r1)
    s2 = math.acos(d2 / r2)
    return s1, s2
def solv_robot_arm2(x, y, z, r1, r2, r3):
    z = z + r3 - j1_z_offset
    Rt = math.sqrt(x**2 + y**2 + z**2)
    Rxy = math.sqrt(x**2 + y**2)
    St = math.asin(z / Rt)
    Sxy = math.atan2(y, x)
    s1, s2 = solv2(r1, r2, Rt)
    sr1 = math.pi / 2 - (s1 + St)
    sr2 = s1 + s2
    sr2_ = sr1 + sr2
    sr3 = math.pi - sr2_
    return Sxy, sr1, sr2, sr3, St, Rt


class PickAndPlace(Node):
    def __init__(self):
        super().__init__('pick_and_place')
        # Action Clients 초기화
        self.arm_action_client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self.gripper_action_client = ActionClient(self, GripperCommand, '/gripper_controller/gripper_cmd')
        ### pick and place ###
        self.initial_position_deg = [0.0, -16.79, -12.04, 121.03]  # 초기
        self.avoid_path1_position_deg = [0.0, -16.79, -12.04, 121.03] # 중간 1
        # self.avoid_path2_position_deg = [-0.0015, -0.293, -0.21, 2.11] # 중간 2
        self.place_joint_positions_deg = [-89.82, 53.70, -66.45, 105.56]  # 최종
        self.z_up_positions_deg = [-89.91, 20.0, -75.5, 107.75]
        ### 보라색 ###
        self.purple_initial_position_deg = [-90.0, -16.79, -12.04, 121.03]
        self.purple_ready_position_deg = [90.0, -16.79, -12.04, 121.03]
        self.purple_avoid_position_deg = [-105, 32.66, -5.96, 64.74]
        ### pick ###
        self.pick_positions_deg = {
            "position1": [16.0, 41.31, -20.48, 71.46], # 좌측 상단
            "position2": [26.46, 16.08, 22.32, 53.26], # 촤측 하단
            "position3": [-16.26, 39.55, -18.19, 73.21], # 우측 상단
            "position4": [-22.06, 19.34, 18.54, 54.58], # 우측 하단
            "purple_pick": [-90.0, 32.66, -5.96, 64.74], # 보라색
            "target_place": [90.0, 32.66, -5.96, 64.74]  # target 내려놓기
        }
        self.count = 0
        self.current_count = 0

        self.pickup = False
        self.back = False
        self.grip = False
        self.start = False
        self.last = False

        self.target_position_deg = None
        self.received_coordinates = False
        self.convey_status = None
        
        self.robot_stop = True


        # JointTrajectory message
        # self.trajectory_msg = JointTrajectory()
        # Sxy, sr1, sr2, sr3, St, Rt = solv_robot_arm2(120.0, 0.0, 90.0, r1, r2, r3)
        # self.trajectory_msg.header = Header()
        # self.trajectory_msg.header.frame_id = ''
        # self.trajectory_msg.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']
        # # print(self.trajectory_msg.points)
        # point = JointTrajectoryPoint()
        # point.positions = [Sxy, sr1 + th1_offset, sr2 + th2_offset, sr3]
        # point.velocities = [0.0] * 4
        # point.accelerations = [0.0] * 4
        # point.time_from_start.sec = 0
        # point.time_from_start.nanosec = 500
        # self.trajectory_msg.points = [point]
        # # print(self.trajectory_msg.points[0].positions[0])
        # goal_msg = FollowJointTrajectory.Goal()
        # goal_msg.trajectory = self.trajectory_msg
        # # print(goal_msg.trajectory,point)
        # self.arm_action_client.wait_for_server()
        # self.arm_action_client.send_goal_async(goal_msg)





        # 시작하자마자 초기위치
        self.move_to_initial_position()
        self.control_gripper(0.019)

        self.get_logger().info("시작하자~~~")
##########################################################
        self.object_subscription = self.create_subscription(
            Point,
            '/object_coordinates',
            self.process_coordinates,
            10
        )
        self.convey_pub = self.create_publisher(
            String,
            '/status_convey',
            10
        )
        self.count_sub = self.create_subscription(
            String,
            'count',
            self.count_callback,
            10
        )
        self.convey_sub = self.create_subscription(
            String,
            '/status_robot',
            self.convey_callback,
            10
        )
        self.marker_distance_subscriber = self.create_subscription(
            String,
            '/distance',
            self.go_robot_callback,
            10
        )

        self.target_distance_sub = self.create_subscription(
            String,
            'target_distance',
            self.go_target_callback,
            10
        )

        self.item_sub = self.create_subscription(
            String,
            'Item',
            self.item_callback,
            10
        )

        self.go_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.target_pub = self.create_publisher(
            String,
            'purple_finish',
            10
        )

        self.start_pub = self.create_publisher(
            String,
            'start',
            10
        )

        # self.back_publisher = self.create_publisher(
        #     Twist,
        #     '/cmd_vel',
        #     10
        # )
####################################################################3
        # # 뭔가 좌표값 두개로 할수있을듯
        # self.purple_place_subscriber = self.create_subscription(
        #     # 좌표2개#####추가
        #     '/purple_place',
        #     self.purple_go_and_place_callback,
        #     10
        # )
########################################################
    def item_callback(self,msg):
        self.start = True


    def degrees_to_radians(self, degrees):
        return [math.radians(angle) for angle in degrees]
    
    def send_trajectory(self, joint_positions_deg):
        joint_positions_rad = self.degrees_to_radians(joint_positions_deg)
        goal_msg = FollowJointTrajectory.Goal()
        trajectory = JointTrajectory()
        trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']
        point = JointTrajectoryPoint()
        point.positions = joint_positions_rad
        point.time_from_start.sec = 2
        trajectory.points = [point]
        goal_msg.trajectory = trajectory
        self.arm_action_client.wait_for_server()
        self.arm_action_client.send_goal_async(goal_msg)

    def control_gripper(self, position):
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = position
        self.gripper_action_client.wait_for_server()
        self.gripper_action_client.send_goal_async(goal_msg)

    def process_coordinates(self, msg):
        x = msg.x
        y = msg.y
        if x > 6.0 and y < 0.0:
            self.target_position_deg = self.pick_positions_deg["position1"]   # 좌측 상단
        elif x < 6.0 and y < 0.0:
            self.target_position_deg = self.pick_positions_deg["position2"]   # 좌측 하단
        elif x > 6.0 and y > 0.0:
            self.target_position_deg = self.pick_positions_deg["position3"]   # 우측 상단
        elif x < 6.0 and y > 0.0:
            self.target_position_deg = self.pick_positions_deg["position4"]   # 우측 하단
        else:
            self.get_logger().info("yolo 실패")
            return

        self.received_coordinates = True
        #self.pick_and_place(msg)
        self.execute()

    def move_to_initial_position(self):
        self.send_trajectory(self.initial_position_deg)
        time.sleep(2)

    def target_pick(self):
        self.send_trajectory(self.target_position_deg)
        time.sleep(4)
        self.control_gripper(-0.01)
        time.sleep(1)

    def avoid_path(self):
        self.send_trajectory(self.avoid_path1_position_deg)
        time.sleep(2)
        
        
    def convey_place(self):
        self.send_trajectory(self.place_joint_positions_deg)
        time.sleep(4)
        self.control_gripper(0.019)
        time.sleep(1)
        self.send_trajectory(self.z_up_positions_deg)
        time.sleep(1)

    def count_callback(self, msg):
        self.count = int(msg.data)

    def publish_status(self):
        msg = String()
        if self.current_count < self.count - 1:
            msg.data = "short"
            self.current_count += 1
        else:
            msg.data = "start"
            self.get_logger().info("convey 진행중")
        self.convey_pub.publish(msg)
    ###############################

    def convey_callback(self, msg):
        if msg.data == 'finish':
            self.purple_pick_and_up()

        self.get_logger().info("컨베이어 완료")

    def go_target_callback(self, msg):
        distance = float(msg.data)
        twist = Twist()
        
        if distance <= 0.20:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            time.sleep(1)
            if self.last == False:
                self.last = True
                self.send_trajectory(self.pick_positions_deg["target_place"])
                time.sleep(2)
                self.control_gripper(0.019)
                time.sleep(2)
                self.send_trajectory(self.purple_ready_position_deg)
        else:
            twist.linear.x = 0.1
            twist.angular.z = 0.0
            self.get_logger().info("터틀봇 전진")
        
        self.go_publisher.publish(twist)
        self.get_logger().info(f"distance:{distance}")

    def go_robot_callback(self, msg):

        if self.start == True:

            distance = float(msg.data)
            twist = Twist()
            if self.back == False:
                if distance <= 0.455:  # 정지
                    self.back = True
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self.get_logger().info("정지")
                    msg = String()
                    msg.data = 'start'
                    self.start_pub.publish(msg)
                    # self.robot_stop = True
                else:
                    twist.linear.x = 0.1
                    twist.angular.z = 0.0
                    # self.robot_stop = False
                    self.get_logger().info("터틀봇 전진")

            elif self.back == True and self.grip == True:
                if distance >= 1.3:  # 정지
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self.robot_stop = True
                else:
                    twist.linear.x = -0.1
                    twist.angular.z = 0.0
                    self.robot_stop = False
                    self.get_logger().info("터틀봇 후진")

            self.go_publisher.publish(twist)
            self.get_logger().info(f"distance:{distance}")


    def purple_pick_and_up(self):
        self.send_trajectory(self.purple_initial_position_deg)
        time.sleep(3)
        self.send_trajectory(self.pick_positions_deg["purple_pick"])
        time.sleep(3)
        self.control_gripper(-0.05)
        time.sleep(2)
        self.send_trajectory(self.purple_avoid_position_deg)
        time.sleep(2)
        self.send_trajectory(self.purple_initial_position_deg)
        time.sleep(3)
        self.send_trajectory(self.purple_ready_position_deg)
        time.sleep(3)

        msg = String()
        msg.data = 'finish'
        self.target_pub.publish(msg)

    

    def initial_pose(self):
        init_x = 90.0
        init_y = 0.0
        init_z = 120.0
        Sxy, sr1, sr2, sr3, St, Rt = solv_robot_arm2(init_x, init_y, init_z, r1, r2, r3)
        print(Sxy, sr1, sr2)
        try:
            self.trajectory_msg.points[0].positions[0] = Sxy
            self.trajectory_msg.points[0].positions[1] = sr1 + th1_offset
            self.trajectory_msg.points[0].positions[2] = sr2 + th2_offset
            self.trajectory_msg.points[0].positions[3] = sr3
            goal_msg = FollowJointTrajectory.Goal()
            goal_msg.trajectory = self.trajectory_msg
            self.arm_action_client.wait_for_server()
            self.arm_action_client.send_goal_async(goal_msg)
            self.get_logger().info("Go Init pos...")
        except Exception as e:
            self.get_logger().error(f"Error calculating joint angles: {e}")

    def pick_pose(self, msg):
        robot_x = -1 * msg.y
        robot_y = -1 * msg.x
        robot_x += 720
        robot_y += 640
        pick_x = robot_x / 60.0 + 50.0
        pick_y = robot_y / 55.65
        pick_z = -30.0
        Sxy, sr1, sr2, sr3, St, Rt = solv_robot_arm2(pick_x, pick_y, pick_z, r1, r2, r3)
        try:
            self.trajectory_msg.points[0].positions[0] = Sxy
            self.trajectory_msg.points[0].positions[1] = sr1 + th1_offset
            self.trajectory_msg.points[0].positions[2] = sr2 + th2_offset
            self.trajectory_msg.points[0].positions[3] = sr3
            goal_msg = FollowJointTrajectory.Goal()
            goal_msg.trajectory = self.trajectory_msg
            self.arm_action_client.wait_for_server()
            self.arm_action_client.send_goal_async(goal_msg)
            self.get_logger().info(f"picking...")
        except Exception as e:
            self.get_logger().error(f"Error calculating joint angles: {e}")

    def place_pose(self):
        place_x = 0.0
        place_y = 230.0
        place_z = 20.0
        Sxy, sr1, sr2, sr3, St, Rt = solv_robot_arm2(place_x, place_y, place_z, r1, r2, r3)
        try:
            self.trajectory_msg.points[0].positions[0] = Sxy
            self.trajectory_msg.points[0].positions[1] = sr1 + th1_offset
            self.trajectory_msg.points[0].positions[2] = sr2 + th2_offset
            self.trajectory_msg.points[0].positions[3] = sr3
            goal_msg = FollowJointTrajectory.Goal()
            goal_msg.trajectory = self.trajectory_msg
            self.arm_action_client.wait_for_server()
            self.arm_action_client.send_goal_async(goal_msg)
            self.get_logger().info(f"placing...")
        except Exception as e:
            self.get_logger().error(f"Error calculating joint angles: {e}")

    #def ungrip(self):
    #    self.send_gripper_goal(-0.015)
    #    self.get_logger().info("Open gripper")

    #def grip(self):
    #    self.send_gripper_goal(0.01)
    #    self.get_logger().info("Close gripper")

    def pick_and_place(self, msg):
        self.ungrip()
        time.sleep(3)
        self.pick_pose(msg)
        time.sleep(3)
        self.grip()
        time.sleep(3)
        self.initial_pose()
        time.sleep(3)
        self.place_pose()
        time.sleep(3)
#####################################################################3
    # def purple_go_and_place_callback(self, mag):
    #     # 초기 상태에서 좌표값 받고 거기로 가서 같은 동작 수행
    #     # 먼저 내려놓는 상황만 만들어놓자
    #     # 좌표가 두개 오면 좋을듯
    #     # 만약 하나만 주면 False하나 주고
    #     # 마지막 과정때 ture추가하고 if 구문 두개 쓰면 될듯??
    #     x = msg.x
    #     y = msg,y
    #     twist = Twist()
    #     # x y 조건
    #     if x > 0.0 and y > 0.0 :
    #         twist.linear.x = 0.1
    #         twist.angular.y = 0.0
    #     elif x > 0.0 and y < 0.0:
    #         twist.linear.x = 0.1
    #         twist.angular.y = 0.0
    #     else:
    #         twist.linear.x = 0.1
    #         twist.angular.y = 0.0
    #     # place 작업 수행
    #     self.send_trajectory(self.purple_place_position_deg)
    #     time.sleep(2)
    #     self.send_trajectory(self.purple_ready_position_deg)
    #     time.sleep(2)

    def execute(self):
        if not self.received_coordinates:
            self.get_logger().info("좌표값 받는중...")
            return
        self.target_pick()
        self.get_logger().info("Pick Start")
        self.avoid_path()
        self.convey_place()
        self.get_logger().info("Pick-and-Place 작업 완료!")
        self.publish_status()
        self.move_to_initial_position()
        self.grip = True
        print(f"{self.grip}")
        ############################

def main(args=None):
    rclpy.init(args=args)
    node = PickAndPlace()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()