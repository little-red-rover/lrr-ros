import struct
import socket
import typing
import time


class RoverConnection:
    def __init__(self, endpoint):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.settimeout(5.0)
        self.endpoint = endpoint
        try:
            self.socket.connect(self.endpoint)
        except Exception:
            pass

    def recv_packet(self) -> typing.Union[bytes, None]:
        """
        Packets are prefixed with the byte string LRR, followed by the message length in bytes.
        """

        data = None
        try:
            while self.socket.recv(3, socket.MSG_PEEK) != b"LRR":
                self.socket.recv(1)

            assert self.recv_length(3) == b"LRR"
            length = struct.unpack("H", self.recv_length(2))[0]
            data = self.recv_length(length)
        except Exception as e:
            print(f"Rover connection hit an error: {e}. Reconnecting...")
            self.reconnect()

        return data

    def reconnect(self):
        while True:
            try:
                self.socket.close()
                self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.socket.settimeout(5.0)
                self.socket.connect(self.endpoint)
                print("Reconnected!")
                return
            except Exception as e:
                time.sleep(1.0)
                print(f"Error while reconnecting: {e}. Trying again in 1 second...")

    def close(self):
        self.socket.close()

    def recv_length(self, length) -> bytes:
        data = bytearray()
        while len(data) < length:
            data.extend(self.socket.recv(length - len(data)))

        return bytes(data)

    def send(self, msg: bytes):
        try:
            self.socket.sendall(b"LRR")
            self.socket.sendall(len(msg).to_bytes(2, byteorder="little"))
            self.socket.sendall(msg)
        except OSError as e:
            if e.errno == 9:
                # The socket is currently closed
                pass
            else:
                print(f"OSError while sending: {e}")
        except Exception as e:
            print(f"Exception while sending: {e}")
