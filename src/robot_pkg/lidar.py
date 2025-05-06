from threading import Thread, Event
import struct, math, time

from robot_pkg.main import log_handler, can_handler
from robot_pkg.consts import IDs, Variables
from robot_pkg.move import Move
from robot_pkg.in_out import I_O, SensorType


class Lidar:
    _logger = log_handler.get_logger("lidar")
    _thread:Thread = None
    last_detection_time = 0
    beacon_points:list = []
    running:Event = Event()
        
    @classmethod
    def _receive(cls, running:Event):
        lidar_queue = can_handler.msg_receive_queues[IDs.GET_OPPONENT.value]
        detection_queue = can_handler.msg_receive_queues[IDs.GET_DETECTION.value]
        beacon_queue = can_handler.msg_receive_queues[IDs.GET_BEACON.value]

        while running.is_set():
            if len(lidar_queue) > 0:
                lidar_msg = lidar_queue.pop()

                [x, y, theta, speed] = struct.unpack('4f', lidar_msg.data)

                # if speed > 150/1000 and abs(Move.pose.speed) > 150:
                #     s.get_intersection(x, y, theta, speed)
                # Lidar._logger.debug(f"Opponent: x:{x:4.2f}, y:{y:4.2f}, theta:{theta*180/math.pi:4.2f}, speed:{speed:4.2f}")

            if len(beacon_queue) > 0:
                lidar_msg = beacon_queue.pop()
                
                xyd = struct.unpack('12f', lidar_msg.data)
                print(xyd)
                Lidar._logger.info("Beacons:")
                for i in range(0, 12, 3):
                    if round(xyd[i],2) != 0 and round(xyd[i+1],2) != 0:
                        Lidar._logger.info(f"{xyd[i]}, {xyd[i+1]}, {xyd[i+2]}")

             
            if len(detection_queue) > 0:
                lidar_msg = detection_queue.pop()

                if abs(Move.pose.speed) > 20 and not Variables.processing_detection.is_set() and not I_O.sensor_states[SensorType.CINCH.value]:
                    detection_side = struct.unpack('B', lidar_msg.data)[0]
                    if detection_side == 70 and Move.pose.speed > 0: # 'F' - FRONT
                        # Lidar.last_detection_time = time.time()
                        Variables.front_detection.set()
                        Variables.processing_detection.set()
                        Lidar._logger.debug(f"FRONT")
                    elif detection_side == 66 and Move.pose.speed < 0: # 'B' - BACK
                        # Lidar.last_detection_time = time.time()
                        Variables.back_detection.set()
                        Variables.processing_detection.set()
                        Lidar._logger.debug(f"BACK")
                    # else:
                    #     pass
                        # Lidar._logger.debug(f"Unknown detection: {detection_side}")

            # Resets last detection time after 1s if not reset before
            # if (Variables.front_detection.is_set() or Variables.back_detection.is_set()) and \
            #    time.time() - Lidar.last_detection_time > 0.1:  # if 1s have passed from last detection
            #     Variables.front_detection.clear()
            #     Variables.back_detection.clear()

            time.sleep(0.01)  # 10ms

    def get_intersection(self, op_x, op_y, op_theta, op_v):
        self_x, self_y, self_theta, self_v = Move.pose.x, Move.pose.y, Move.pose.theta, Move.pose.speed

        if self_v < 0:
            self_theta += math.pi

            if self_theta > math.pi:
                self_theta -= 2*math.pi
            elif self_theta < -math.pi:
                self_theta += 2*math.pi

        int_x, int_y = self.get_intersection_point(op_x, op_y, op_theta, self_x, self_y, self_theta)
        print(f"INTERSECTION: x={int_x}, y={int_y}")

        # if 0 <= int_x <= 2950 and 0 <= int_y <= 1950:
        self_distance  = self.get_intersection_distance(int_x, int_y, self_x, self_y, self_v)
        op_distance    = self.get_intersection_distance(int_x, int_y, op_x, op_y, op_v)

        print(f"SELF DIST TO INTERSECTION POINT    : {self_distance}")
        print(f"OPPONENT DIST TO INTERSECTION POINT: {op_distance}")
            
    def get_intersection_point(self, op_x, op_y, op_theta, self_x, self_y, self_theta):
        self_k = math.tan(self_theta)
        op_k = math.tan(op_theta)
        self_n = self_y - self_k*self_x
        op_n = op_y - op_k*op_x  

        int_x = (op_n - self_n)/(self_k - op_k + 1e-10)
        int_y = self_k*(int_x - self_x) + self_y

        return int_x, int_y
    
    def get_intersection_distance(self, int_x, int_y, x, y, v):
        return math.sqrt( (int_x - x)**2 + (int_y - y)**2 ) 
        # t = distance/(v + 1e-10)  # mm/(m/s) = ms
        # return t

    @classmethod
    def start_threads(cls, color:str):
        Lidar.start_stop(1, color)
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
    def start_stop(cls, start_stop, color=None):
        send_queue = can_handler.msg_send_queues[IDs.SET_LIDAR.value]
        color_byte = 'y'.encode('utf_8') if color == "yellow" else 'b'.encode('utf_8') if color == "blue" else '0'.encode('utf_8')
        lidar_msg = struct.pack('Bc', (start_stop & 0x01), color_byte)
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