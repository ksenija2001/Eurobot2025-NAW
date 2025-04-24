import socket
from threading import Thread
import time, struct

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

        self.connections = {1: None, 2:None, 3:None, 4:None}
        self.addresses   = {1: None, 2:None, 3:None, 4:None}

        self.running = False
        self._thread = Thread(target=self.accept_connections)

    def start_threads(self):
        self.running = True

        self.s.listen()
        self.s.settimeout(0.5)

        self._thread.start()

        self._logger.info("Started listening for sima")

    def send_command(self, sima_ID:int, coordinates:list[Position]):
        size = len(coordinates)
        packed = [size]
        for position in coordinates:
            packed.extend([position.x, position.y, position.theta])

        data = struct.pack('f'*size*3, *packed)
        self.connections[sima_ID].send(data)

        self._logger.info(f"Sending coordinates to SIMA {sima_ID}")

    def accept_connections(self):
        while self.running and any(self.connections) is None:
            for connection in self.connections:
                if connection is None:
                    try:
                        conn, address = self.s.accept()
                        # blocks until one byte that contains the ID of the connected SIMA is received
                        ID = self.s.recv(1) 
                        self.addresses[ID] = address
                        self.connections[ID] = conn

                        self._logger.info(f"SIMA {ID} CONNECTED")
                    except:
                        pass
        
    def stop(self):
        self.running = False
        if self.thread.is_alive():
            self.thread.join()

        for connection in self.connections:
            if connection is not None:
                connection.close()
      
        self.s.detach()
        self.s.close()
        self._logger.info("SIMA thread stopped.")

if __name__ == "__main__":
    sima = SIMA()

