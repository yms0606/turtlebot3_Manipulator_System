from __future__ import print_function
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Point
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge
import numpy as np
import sys


desired_aruco_dictionary = "DICT_5X5_100"

GOAL_ID = 33
ROBOT_ID = 20
TARGET_ID = [22,43,24]
# 0.04 -0.2 0.6
# -0.04 -0.05 0.4
ARUCO_DICT = {
  "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
  "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
  "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
  "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
  "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
  "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
  "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
  "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
  "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
  "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
  "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
  "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
  "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
  "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
  "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
  "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
  "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL
}

# ls -al /dev/video*


class CamNode(Node):

    def __init__(self):
        super().__init__('camnode')

        self.cap = cv2.VideoCapture(2,cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        #self.cap.set(cv2.CAP_PROP_FPS, 25)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        
        if not self.cap.isOpened():
            self.get_logger().error("Failed to open camera!")
            return
        
        ################ calibration ############################
        if ARUCO_DICT.get(desired_aruco_dictionary, None) is None:
            print("ERROR")
            sys.exit(0)

        self.this_aruco_dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[desired_aruco_dictionary])
        self.this_aruco_parameters = cv2.aruco.DetectorParameters_create()
        self.k = np.load('/home/yms/rokey_week8_ws/src/smart_factory/smart_factory/mtx720_12301.npy')
        self.d = np.load('/home/yms/rokey_week8_ws/src/smart_factory/smart_factory/dist720_12301.npy')

        ##########################################################

        self.bridge = CvBridge()
        self.pub_img = self.create_publisher(CompressedImage,'global_img',10)
        self.pub_pose = self.create_publisher(String,'re_pose',10)
        self.pub_zpose = self.create_publisher(String,'/distance',10)
        self.pub_targetpose = self.create_publisher(String, 'target_distance',10)

        self.goTarget = False
        self.target = None

        self.sub_finish = self.create_subscription(String,'purple_finish',self.finish_callback,10)
        self.sub_target = self.create_subscription(String,'Item',self.item_callback,10)

        self.timer = self.create_timer(0.01,self.timer_callback)
    

    def finish_callback(self,msg):
        if msg.data == 'finish':
            self.goTarget = True

    def item_callback(self, msg):
        item = msg.data.split()[-1]
        self.target = int(item.split('*')[1]) - 1
    

    def compute_relative_pose(self,goal_pose, robot_pose):

        t_goal = goal_pose[0:3]
        rvec_goal = goal_pose[3:]

        R_goal, _ = cv2.Rodrigues(rvec_goal)

        T_goal = np.eye(4)
        T_goal[:3, :3] = R_goal
        T_goal[:3, 3] = t_goal

        T_goal_inv = np.linalg.inv(T_goal)

        t_robot= robot_pose[:3] 
        rvec_robot = robot_pose[3:]
        R_robot, _ = cv2.Rodrigues(rvec_robot)

        T_robot = np.eye(4)
        T_robot[:3, :3] = R_robot
        T_robot[:3, 3] = t_robot

        T_relative = T_goal_inv @ T_robot

        R_relative = T_relative[:3, :3]
        t_relative = T_relative[:3, 3]

        rvec_relative, _ = cv2.Rodrigues(R_relative)

        return t_relative, rvec_relative


    def timer_callback(self):
        ret,frame = self.cap.read()

        #print(frame.shape)

        #################################################################
        ########################## calibration ##########################
        #################################################################

        (corners, ids, rejected) = cv2.aruco.detectMarkers(
            frame, self.this_aruco_dictionary, parameters=self.this_aruco_parameters
        )

        goal_tvec = None
        goal_rvec = None
        cal_tvec = None
        cal_rvec = None
        target_tvec = None
        target_rvec = None

        if len(corners) > 0:
            ids = ids.flatten()

            for (marker_corner, marker_id) in zip(corners, ids):

                corners = marker_corner.reshape((4,2))
                (top_left, top_right, bottom_right, bottom_left) = corners

                top_right = (int(top_right[0]), int(top_right[1]))
                bottom_right = (int(bottom_right[0]),int(bottom_right[1]))
                bottom_left = (int(bottom_left[0]),int(bottom_left[1]))
                top_left = (int(top_left[0]),int(top_left[1]))

                cv2.line(frame, top_left, top_right, (0,255,0),2)
                cv2.line(frame, top_right, bottom_right, (0,255,0),2)
                cv2.line(frame, bottom_right, bottom_left, (0,255,0),2)
                cv2.line(frame, bottom_left, top_left, (0,255,0),2)


                center_x = int((top_left[0] + bottom_right[0])/ 2.0)
                center_y = int((top_left[1] + bottom_right[1])/ 2.0)

                #cv2.circle(frame, (center_x,center_y),4,(0,0,255), -1)

                cv2.putText(frame, str(marker_id),
                            (top_left[0], top_left[1] -15),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.5,(0,255,0),2)
                
                rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(marker_corner, 0.1065,self.k, self.d)
                cv2.drawFrameAxes(frame, self.k, self.d, rvec, tvec, 0.03)

                #if marker_id == 20 or marker_id==33:# or marker_id==80:#or marker_id == 43 or marker_id==86 or marker_id==80:
                #    print(f"{marker_id}: {tvec}, {rvec}")
                

                if marker_id == GOAL_ID: # 33
                    goal_tvec = tvec[0][0]
                    goal_rvec = rvec[0][0]
                elif marker_id == ROBOT_ID: # 20
                    cal_tvec = tvec[0][0]
                    cal_rvec = rvec[0][0]
                elif self.goTarget == True and marker_id == TARGET_ID[self.target]:
                    target_tvec = tvec[0][0]
                    target_rvec = rvec[0][0]
                #else:
                #    self.get_logger().warn('no robot in cam!')
        #################################################################
        #################################################################
        #################################################################

            if ROBOT_ID in ids and GOAL_ID in ids:
                goal_pose = np.concatenate((goal_tvec, goal_rvec))
                robot_pose = np.concatenate((cal_tvec, cal_rvec))
                re_tvec, re_rvec = self.compute_relative_pose(goal_pose,robot_pose) 
                
                re_msg = String()
                re_msg.data = f"coor: {re_tvec} \naxis: {re_rvec}"
                self.pub_pose.publish(re_msg)

                if self.goTarget == False:
                    msg = String()
                    msg.data = f"{re_tvec[2]}"
                    self.pub_zpose.publish(msg)
                
                elif self.goTarget == True and TARGET_ID[self.target] in ids:
                    
                    target_pose = np.concatenate((target_tvec,target_rvec))
                    target_re_tvec, target_re_rvec = self.compute_relative_pose(goal_pose,target_pose)
                    
                    msg = String()
                    msg.data = f"{re_tvec[2] - target_re_tvec[2]}"
                    self.pub_targetpose.publish(msg)

        self.pub_img.publish(self.bridge.cv2_to_compressed_imgmsg(frame))


if __name__ == '__main__':
    rclpy.init()

    node = CamNode()

    rclpy.spin(node)

    rclpy.shutdown()
    node.destroy_node()
    
