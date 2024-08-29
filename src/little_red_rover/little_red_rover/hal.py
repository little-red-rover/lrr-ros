import rospy

from math import floor, inf, pi

from sensor_msgs.msg import JointState
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

import threading
import socket

import little_red_rover.pb.messages_pb2 as messages


class HAL:
    def __init__(self):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind(("0.0.0.0", 8001))

        self.joint_state_publisher = rospy.Publisher(
            "joint_states", JointState, queue_size=10
        )
        self.scan_publisher = rospy.Publisher("scan", LaserScan, queue_size=10)

        self.laser_msg = LaserScan()
        self.laser_msg.header.frame_id = "lidar"
        self.laser_msg.range_min = 0.1
        self.laser_msg.range_max = 8.0
        self.laser_msg.angle_min = 0.0
        self.laser_msg.angle_max = 2.0 * pi
        self.laser_msg.ranges = [0.0] * 720
        self.laser_msg.intensities = [0.0] * 720
        self.laser_msg.angle_increment = (2 * pi) / (len(self.laser_msg.ranges))

        threading.Thread(target=self.run_loop).start()

    def run_loop(self):
        while not rospy.is_shutdown():
            data = self.socket.recv(1500)
            packet = messages.UdpPacket()
            try:
                packet.ParseFromString(data)
            except Exception as e:
                print(e)
                continue

            if packet.HasField("laser"):
                self.handle_laser_scan(packet.laser)
            elif packet.HasField("joint_states"):
                self.handle_joint_states(packet.joint_states)

    def handle_joint_states(self, packet: messages.JointStates):
        msg = JointState()
        msg.header.frame_id = "robot_body"
        # msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.stamp.set(packet.time.sec, packet.time.nanosec)
        msg.name = list(packet.name)
        msg.effort = packet.effort
        msg.position = packet.position
        msg.velocity = packet.velocity
        self.joint_state_publisher.publish(msg)

    def handle_laser_scan(self, packet: messages.LaserScan):
        pass

    def cmd_vel_callback(self, msg: Twist):
        pass


def main(args=None):
    rospy.init_node("hal", anonymous=True)
    hal_node = HAL()

    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
