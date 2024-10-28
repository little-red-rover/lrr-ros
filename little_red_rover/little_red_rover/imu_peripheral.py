import rospy
from sensor_msgs.msg import Imu
import little_red_rover.pb.messages_pb2 as messages


class ImuPeripheral:
    def __init__(self):
        self.imu_publisher = rospy.Publisher("imu/data_raw", Imu, queue_size=10)
        self.imu_msg = Imu()
        self.imu_msg.header.frame_id = "robot_body"

    def handle_packet(self, packet: messages.NetworkPacket):
        if not packet.HasField("imu"):
            return
        self.imu_msg.header.stamp.set(packet.imu.time.sec, packet.imu.time.nanosec)

        # disable orientation
        self.imu_msg.orientation_covariance = [-1.0] + [0.0] * 8

        # accel
        self.imu_msg.linear_acceleration.x = packet.imu.accel_x
        self.imu_msg.linear_acceleration.y = packet.imu.accel_y
        self.imu_msg.linear_acceleration.z = packet.imu.accel_z
        self.imu_msg.linear_acceleration_covariance = [0.0] * 9  # TODO

        # gyro
        self.imu_msg.angular_velocity.x = packet.imu.gyro_x
        self.imu_msg.angular_velocity.y = packet.imu.gyro_y
        self.imu_msg.angular_velocity.z = packet.imu.gyro_z
        self.imu_msg.angular_velocity_covariance = [0.0] * 9  # TODO

        self.imu_publisher.publish(self.imu_msg)
