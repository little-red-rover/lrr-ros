import rospy

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist

import little_red_rover.pb.messages_pb2 as messages
from little_red_rover.rover_connection import RoverConnection


class DriveBasePeripheral:
    def __init__(self, connection: RoverConnection):
        self.joint_state_publisher = rospy.Publisher(
            "joint_states", JointState, queue_size=10
        )

        self.joint_state_msg = JointState()
        self.joint_state_msg.header.frame_id = "robot_body"
        self.subscription = rospy.Subscriber("cmd_vel", Twist, self.cmd_vel_callback)

        self.connection = connection

    def handle_packet(self, packet: messages.NetworkPacket):
        if not packet.HasField("joint_states"):
            return

        self.joint_state_msg.header.stamp.set(
            packet.joint_states.time.sec, packet.joint_states.time.nanosec
        )
        self.joint_state_msg.name = list(packet.joint_states.name)
        self.joint_state_msg.effort = packet.joint_states.effort
        self.joint_state_msg.position = packet.joint_states.position
        self.joint_state_msg.velocity = packet.joint_states.velocity
        self.joint_state_publisher.publish(self.joint_state_msg)

    def cmd_vel_callback(self, msg: Twist):
        packet = messages.NetworkPacket()
        packet.cmd_vel.v = msg.linear.x
        packet.cmd_vel.w = msg.angular.z

        self.connection.send(packet.SerializeToString())
