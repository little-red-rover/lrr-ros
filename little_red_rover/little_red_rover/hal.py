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

        self.subscription = rospy.Subscriber("cmd_vel", Twist, self.cmd_vel_callback)

        self.joint_state_publisher = rospy.Publisher(
            "joint_states", JointState, queue_size=10
        )
        self.scan_publisher = rospy.Publisher("scan", LaserScan, queue_size=10)

        self.ranges = [0.0] * 720
        self.intensities = [0.0] * 720

        threading.Thread(target=self.run_loop).start()

    def run_loop(self):
        laser_msg = LaserScan()
        laser_msg.header.frame_id = "lidar"
        laser_msg.range_min = 0.1
        laser_msg.range_max = 8.0
        laser_msg.angle_min = 0.0
        laser_msg.angle_max = 2.0 * pi
        laser_msg.angle_increment = (2 * pi) / (len(self.ranges))

        joint_state_msg = JointState()
        joint_state_msg.header.frame_id = "robot_body"

        while not rospy.is_shutdown():
            data = self.socket.recv(1500)
            packet = messages.UdpPacket()
            try:
                packet.ParseFromString(data)
            except Exception as e:
                print(e)
                continue

            if packet.HasField("laser"):
                self.handle_laser_scan(laser_msg, packet.laser)
            elif packet.HasField("joint_states"):
                self.handle_joint_states(joint_state_msg, packet.joint_states)

    def handle_joint_states(self, msg: JointState, packet: messages.JointStates):
        msg.header.stamp.set(packet.time.sec, packet.time.nanosec)
        msg.name = list(packet.name)
        msg.effort = packet.effort
        msg.position = packet.position
        msg.velocity = packet.velocity
        self.joint_state_publisher.publish(msg)

    def handle_laser_scan(self, msg: LaserScan, packet: messages.LaserScan):
        break_in_packet = False

        for i in range(len(packet.ranges)):
            angle = packet.angle_min + (packet.angle_max - packet.angle_min) * (
                i / (len(packet.ranges) - 1)
            )
            index = int(((angle % (2.0 * pi)) / (2.0 * pi)) * 720.0)

            if angle > pi * 2.0 and not break_in_packet:
                msg.time_increment = packet.time_increment
                msg.scan_time = packet.scan_time
                msg.ranges = self.ranges
                msg.intensities = self.intensities

                break_in_packet = True
                self.scan_publisher.publish(msg)
                self.ranges = [0.0] * 720
                self.intensities = [0.0] * 720

                msg.header.stamp.set(packet.time.sec, packet.time.nanosec)
                # msg.header.stamp = self.get_clock().now().to_msg()

            self.ranges[index] = packet.ranges[i]
            self.intensities[index] = packet.intensities[i]

            if (
                self.ranges[index] > 8.0
                or self.ranges[index] < 0.1
                or self.intensities[index] == 0
            ):
                self.ranges[index] = inf
                self.intensities[index] = 0.0

    def cmd_vel_callback(self, msg: Twist):
        packet = messages.UdpPacket()
        packet.cmd_vel.v = msg.linear.x
        packet.cmd_vel.w = msg.angular.z

        self.socket.sendto(packet.SerializeToString(), ("192.168.4.1", 8001))


def main(args=None):
    rospy.init_node("hal", anonymous=True)
    _ = HAL()

    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
