from threading import Thread, Event
import struct, math, time

from robot_pkg.main import log_handler, can_handler
from robot_pkg.consts import IDs


class Lidar:
    _logger = log_handler.get_logger("lidar")
    _thread:Thread = None
    running:Event = Event()
        
    @classmethod
    def _receive(cls, running:Event):
        lidar_queue = can_handler.msg_receive_queues[IDs.GET_LIDAR.value]
        while running.is_set():
            if len(lidar_queue) > 0:
                lidar_msg = lidar_queue.pop()

                [x, y, theta, timestamp] = struct.unpack('4f', lidar_msg.data)

                Lidar._logger.debug(f"Opponent [{timestamp}]: x:{x:4.2f}, y:{y:4.2f}, theta:{theta*180/math.pi:4.2f}")

            time.sleep(0.01)  # 10ms
    
    @classmethod
    def start_threads(cls):
        Lidar.start_stop(1)
        Lidar.running.set()
        if Lidar._thread is None:
            Lidar._thread = Thread(target=Lidar._receive, args=(Lidar.running, ))
            Lidar._thread.start()
        Lidar._logger.info("Lidar receiving thread started.")

    @classmethod
    def stop_threads(cls):
        Lidar.start_stop(0)
        Lidar.running.clear()
        if Lidar._thread is not None and Lidar._thread.is_alive():
            Lidar._thread.join()
        Lidar._thread = None
        Lidar._logger.info("Lidar receiving thread stopped.")

    @classmethod
    def start_stop(cls, start_stop):
        send_queue = can_handler.msg_send_queues[IDs.SET_LIDAR.value]
        lidar_msg = struct.pack('B', (start_stop & 0x01))
        send_queue.append(lidar_msg)

if __name__ == "__main__":
    can_handler.start_threads()

    Lidar.start_threads()

    start_time = time.time()
    while(1):
        if time.time() - start_time > 600:
            break

    Lidar.stop_threads()

    can_handler.stop_threads()