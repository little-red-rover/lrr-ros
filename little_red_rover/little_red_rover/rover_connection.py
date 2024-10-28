import struct
import socket


class RoverConnection:
    def __init__(self, endpoint):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.settimeout(5.0)
        self.endpoint = endpoint
        self.socket.connect(endpoint)

    def recv_packet(self) -> bytes:
        """
        Packets are prefixed with the byte string LRR, followed by the message length in bytes.
        """

        data = None
        while data == None:
            try:
                while self.socket.recv(3, socket.MSG_PEEK) != b"LRR":
                    self.socket.recv(1)

                assert self.recv_length(3) == b"LRR"
                length = struct.unpack("H", self.recv_length(2))[0]
                data = self.recv_length(length)
            except Exception as e:
                print(f"Rover connection hit error: {e}. Reconnecting...")
                self.socket.close()
                self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.socket.settimeout(5.0)
                self.socket.connect(self.endpoint)
                print(f"Error: {e}")

        return data

    def recv_length(self, length) -> bytes:
        data = bytearray()
        while len(data) < length:
            data.extend(self.socket.recv(length - len(data)))

        return bytes(data)

    def send(self, msg: bytes):
        try:
            self.socket.sendall(msg)
        except OSError as e:
            if e.errno == 9:
                # The socket is currently closed
                pass
            else:
                print(f"OSError while sending: {e}")
        except Exception as e:
            print(f"Exception while sending: {e}")
