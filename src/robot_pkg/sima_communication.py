import socket
from threading import Thread
import time
import struct

from robot_pkg.main import log_handler
from robot_pkg.move import Position
from robot_pkg.consts import IP


class SIMA:
    def __init__(self):
        self._logger = log_handler.get_logger("sima")

        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        try:
            self.s.bind((IP, 9999))
        except:
            self.s.close()
            self.s.bind((IP, 9999))

        self._logger.info("Opened port")

        self.connections = {1: None, 2: None, 3: None, 4: None}
        self.addresses = {1: None, 2: None, 3: None, 4: None}

        self.running = False
        self._thread = Thread(target=self.accept_connections)

        self.coordinates = {1: None, 2: None, 3: None, 4: None}

        self.sent = False

    def start_threads(self):
        self.running = True

        self.s.listen()
        self.s.settimeout(0.5)

        self._thread.start()

        self._logger.info("Started listening for sima")

    def send_start(self):
        try:
            s = 1
            data = struct.pack("B", s)
            for id, connection in self.connections.items():
                if connection is not None:
                    connection.send(data)
        except Exception as e:
            print(e)
            pass

        self.sent = True
        self._logger.info(f"Sending start to all connected SIMA")

    def send_command(self, sima_ID: int, coordinates: list[Position]):
        size = len(coordinates)
        packed = [size]
        for position in coordinates:
            packed.extend(
                [position.x, position.y, position.theta, position.speed])

        try:
            data = struct.pack('<B'+'f'*(size*4), *packed)
            self.connections[sima_ID].send(data)
        except Exception as e:
            print(e)
            pass

        self._logger.info(f"Sending coordinates to SIMA {sima_ID}: {data}")

    def accept_connections(self):
        while self.running and any([True for _, connection in self.connections.items() if connection is None]):
            try:
                conn, address = self.s.accept()
                # blocks until one byte that contains the ID of the connected SIMA is received
                ID = int(conn.recv(1))
                self.addresses[ID] = address
                self.connections[ID] = conn

                self._logger.info(f"SIMA {ID} CONNECTED")

                self.send_command(ID, self.coordinates[ID])
            except Exception as e:
                # print(e)
                pass

            time.sleep(0.1)

    def stop_threads(self):
        self.running = False
        if self._thread.is_alive():
            self._thread.join()

        for _, connection in self.connections.items():
            if connection is not None:
                connection.close()

        self.s.detach()
        self.s.close()
        self._logger.info("SIMA thread stopped.")


if __name__ == "__main__":
    sima = SIMA()
