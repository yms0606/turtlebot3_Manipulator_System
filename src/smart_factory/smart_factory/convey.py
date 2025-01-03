import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import threading
import smtplib
import serial
import sys


# ls -al /dev/ttyACM*
# sudo chmod 666 /dev/ttyACM3

class ConveyNode(Node):

    def __init__(self):
        super().__init__('conveynode')
        
        self.email = 'ymsix0622@gmail.com'
        self.pwd = 'plgh wfgm nmie hejn'

        self.port = '/dev/ttyACM1'
        self.baundrate= 115200

        self.Ismoving = False
        self.signal = None

        self.target = None

        self.ser = serial.Serial(port=self.port,baudrate=self.baundrate,timeout=1)

        threading.Thread(target=self.moving_signal,daemon=True).start()

        self.sub_start = self.create_subscription(String,'status_convey',self.status_callback,10)
        
        self.pub_sccuess = self.create_publisher(String, 'IsSuccess',10)

        self.pub_start = self.create_publisher(String,'status_robot',10)

        self.pub_target = self.create_publisher(String,'target',10)


    def status_callback(self,msg):

        self.signal = msg.data

        self.get_logger().info(f"msg is {self.signal}")

        if self.signal == 'start':
            self.ser.write(b'10000\n')
            self.Ismoving = True

        elif self.signal == 'short':
            self.ser.write(b'1000\n')
            self.Ismoving = True

        elif self.signal == 'stop':
            self.ser.write(b'1\n')
            self.Ismoving = False


    def send_mail(self):
        connection = smtplib.SMTP("smtp.gmail.com") 
        connection.starttls()
        connection.login(user=self.email, password=self.pwd)
        connection.sendmail(
                from_addr="warning@warning.ing",
                to_addrs="jang4660893@naver.com",
                msg="Subject:ERRORRRRRRR!!!\n\nYour conveyorbelt is effeted by something.\n\n\nPlease check your belt."
            )                                           
        connection.close()



    def moving_signal(self):
        while True:
            try:
                if self.ser.in_waiting > 0:
                    rx = self.ser.read().decode('ascii')
                    if self.Ismoving and rx =='.':
                        self.Ismoving = False

                        if self.signal == 'start': 
                            self.get_logger().info('success')
                            msg1 = String()
                            msg1.data = 'finish'
                            self.pub_start.publish(msg1)

                        msg = String()
                        msg.data = 'success'
                        self.pub_sccuess.publish(msg)

            except OSError:
                self.send_mail()
                self.get_logger().error('cannot connect conveyor belt')
                sys.exit(0)
                break


if __name__ == '__main__':
    rclpy.init()

    node = ConveyNode()

    rclpy.spin(node)

    rclpy.shutdown()
    node.destroy_node()