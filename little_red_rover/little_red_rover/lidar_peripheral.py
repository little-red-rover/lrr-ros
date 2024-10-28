import rospy
from sensor_msgs.msg import LaserScan
import little_red_rover.pb.messages_pb2 as messages

from math import inf, pi


class LidarPeripheral:
    def __init__(self):
        self.ranges = [0.0] * 720
        self.intensities = [0.0] * 720

        self.scan_publisher = rospy.Publisher("scan", LaserScan, queue_size=10)

        self.laser_msg = LaserScan()
        self.laser_msg.header.frame_id = "lidar"
        self.laser_msg.range_min = 0.1
        self.laser_msg.range_max = 8.0
        self.laser_msg.angle_min = 0.0
        self.laser_msg.angle_max = 2.0 * pi
        self.laser_msg.angle_increment = (2 * pi) / (len(self.ranges))

    def handle_packet(self, packet: messages.NetworkPacket):
        if len(packet.laser) == 0:
            return
        for scan in packet.laser:
            self.handle_laser_scan(self.laser_msg, scan)
        pass

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

                self.scan_publisher.publish(msg)
                break_in_packet = True

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
