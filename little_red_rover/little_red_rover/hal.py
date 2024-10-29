from google.protobuf.message import DecodeError
import rospy

import threading
import little_red_rover.pb.messages_pb2 as messages

from little_red_rover.rover_connection import RoverConnection
from little_red_rover.lidar_peripheral import LidarPeripheral
from little_red_rover.drive_base_peripheral import DriveBasePeripheral
from little_red_rover.imu_peripheral import ImuPeripheral


class HAL:
    def __init__(self):
        self.connection = RoverConnection(("192.168.4.1", 8001))

        self.peripherals = [
            LidarPeripheral(),
            DriveBasePeripheral(self.connection),
            ImuPeripheral(),
        ]

        self.decode_error_count = 0

        threading.Thread(target=self.run_loop, daemon=True).start()

    def run_loop(self):
        while not rospy.is_shutdown():
            try:
                data = None
                while data == None:
                    try:
                        data = self.connection.recv_packet()
                    except Exception as e:
                        print(e)

                packet = messages.NetworkPacket()
                packet.ParseFromString(bytes(data))

                for peripheral in self.peripherals:
                    peripheral.handle_packet(packet)

            except DecodeError:
                self.decode_error_count += 1
            except Exception as e:
                print(f"HAL: Error - {e}")

        self.connection.close()


def main(_=None):
    rospy.init_node("hal", anonymous=True)
    _ = HAL()

    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
