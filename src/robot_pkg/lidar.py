from threading import Thread, Event
import struct
import math
import time
import socket

from robot_pkg.main import log_handler, can_handler
from robot_pkg.consts import IDs, Variables, IP
from robot_pkg.move import Move, Position
from robot_pkg.in_out import I_O, SensorType
from robot_pkg.play_elements import Area, MaterialStack


class Opponent:
    def __init__(self, x: float = 0, y: float = 0, theta: float = 0, speed: float = 0):
        self.pose = Position(x, y, theta)

        self._logger = log_handler.get_logger("opponent")

        self.last_area = None
        self.area_entry_time = None
        self.last_stack = None
        self.stack_entry_time = None

        try:
            self.pc_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.pc_socket.setsockopt(
                socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.pc_socket.connect(('localhost', 8089))

            self._logger.info("Computer connected")
        except Exception as e:
            print(e)

    def _send_opponent_info(self, data):
        try:
            if self.pc_socket is not None:
                self.pc_socket.send(data)
        except Exception as e:
            print(e)
            pass

    def update_position(self, x: float, y: float, theta: float, speed: float):
        """
        Update opponent's current position and check if it is in any area or stack
        """
        self.pose.reset(x, y, theta, speed)

        self._check_area_occupation()
        self._check_stack_occupation()

    def _check_area_occupation(self):
        """
        Check if opponent is in any defined area and mark as visited if stays for more than 2 seconds
        """

        current_area = None
        area_name = None

        for name, area in Area.get_all_areas():
            if (area.x - 225 <= self.x <= area.x + 225 and
                    area.y - 225 <= self.y <= area.y + 225):

                current_area = area
                area_name = name
                break

        if current_area != self.last_area:
            # Opponent moved to a new area or left the previous one
            self.last_area = current_area
            self.area_entry_time = time.time() if current_area else None
        elif current_area is not None and (time.time() - self.area_entry_time > 2):
            # Opponent has been in this area for more than 2 seconds
            if not current_area.visited:
                current_area.visited = True

                data = ['A', area_name]
                data = struct.pack('cs', data)
                self._send_opponent_info(data)

                self._logger.debug(
                    f"Opponent marked area {current_area} as visited")

    def _check_stack_occupation(self):
        """
        Check if opponent is near any stack and mark as visited if stays for more than two seconds
        """
        current_stack: MaterialStack = None
        stack_name = None

        for name, stack in MaterialStack.get_all_unvisited_stacks():
            # 200 mm radius
            if stack.distance_to(self.x, self.y) < 200:
                current_stack = stack
                stack_name = name
                break

        if current_stack != self.last_stack:
            # Opponent moved to a new stack or left the previous one
            self.last_stack = current_stack
            self.stack_entry_time = time.time() if current_stack else None
        elif current_stack is not None and (time.time() - self.stack_entry_time > 2):
            # Opponent has been near this stack for more than 2 seconds
            if not current_stack.visited:
                current_stack.visited = True

                data = ['S', stack_name]
                data = struct.pack('cs', data)
                self._send_opponent_info(data)
                # You might want to log this event
                self._logger.debug(
                    f"Opponent marked stack {current_stack} as visited")


class Lidar:
    _logger = log_handler.get_logger("lidar")
    _thread: Thread = None
    running: Event = Event()
    last_detection_time = 0
    opponent = Opponent()
    # visualizer = FieldVisualizer() negde u glavnom threadu ili ne znam iskreno gde

    @classmethod
    def _receive(cls, running: Event):
        lidar_queue = can_handler.msg_receive_queues[IDs.GET_OPPONENT.value]
        detection_queue = can_handler.msg_receive_queues[IDs.GET_DETECTION.value]

        while running.is_set():
            if len(lidar_queue) > 0:
                lidar_msg = lidar_queue.pop()

                [x, y, theta, speed] = struct.unpack('4f', lidar_msg.data)

                Lidar.opponent.update_position(x, y, theta, speed)

                msg = bytes(lidar_msg.data[0:12])
                data = ['O']
                data.extend(msg)

                Lidar.opponent._send_opponent_info(data)

                # if speed > 150/1000 and abs(Move.pose.speed) > 150:
                #     s.get_intersection(x, y, theta, speed)
                # Lidar._logger.debug(f"Opponent: x:{x:4.2f}, y:{y:4.2f}, theta:{theta*180/math.pi:4.2f}, speed:{speed:4.2f}")

                # Moved to visualizer
                # opponent.update_position(x, y, theta, speed)

            # if len(beacon_queue) > 0:
            #     lidar_msg = beacon_queue.pop()

            #     xyd = struct.unpack('12f', lidar_msg.data)
            #     print(xyd)
            #     Lidar._logger.info("Beacons:")
            #     for i in range(0, 12, 3):
            #         if round(xyd[i], 2) != 0 and round(xyd[i+1], 2) != 0:
            #             Lidar._logger.info(f"{xyd[i]}, {xyd[i+1]}, {xyd[i+2]}")

            if len(detection_queue) > 0:
                lidar_msg = detection_queue.pop()

                if time.time() - Lidar.last_detection_time > 1 and not Variables.processing_detection.is_set() and not I_O.sensor_states[SensorType.CINCH.value]:
                    detection_side = struct.unpack('B', lidar_msg.data)[0]

                    # 'F' - FRONT
                    if detection_side == 70 and Move.detection_enabled['front']:
                        Lidar.last_detection_time = time.time()
                        Variables.front_detection.set()
                        Variables.processing_detection.set()
                        Lidar._logger.debug(f"FRONT")
                    # 'B' - BACK
                    elif detection_side == 66 and Move.detection_enabled['back']:
                        Lidar.last_detection_time = time.time()
                        Variables.back_detection.set()
                        Variables.processing_detection.set()
                        Lidar._logger.debug(f"BACK")

            # From Move.pose for robot, and Lidar data for opponent, MOVED TO VISUALIZER
            # visualizer.update_positions(
            #     robot_x=Move.pose.x,
            #     robot_y=Move.pose.y,
            #     robot_theta=Move.pose.theta,
            #     opponent_x=opponent.x,
            #     opponent_y=opponent.y,
            #     opponent_theta=opponent.theta
            # )

            time.sleep(0.01)  # 10ms

    def get_intersection(self, op_x, op_y, op_theta, op_v):
        self_x, self_y, self_theta, self_v = Move.pose.x, Move.pose.y, Move.pose.theta, Move.pose.speed

        if self_v < 0:
            self_theta += math.pi

            if self_theta > math.pi:
                self_theta -= 2*math.pi
            elif self_theta < -math.pi:
                self_theta += 2*math.pi

        int_x, int_y = self.get_intersection_point(
            op_x, op_y, op_theta, self_x, self_y, self_theta)
        print(f"INTERSECTION: x={int_x}, y={int_y}")

        # if 0 <= int_x <= 2950 and 0 <= int_y <= 1950:
        self_distance = self.get_intersection_distance(
            int_x, int_y, self_x, self_y, self_v)
        op_distance = self.get_intersection_distance(
            int_x, int_y, op_x, op_y, op_v)

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
        return math.sqrt((int_x - x)**2 + (int_y - y)**2)
        # t = distance/(v + 1e-10)  # mm/(m/s) = ms
        # return t

    @classmethod
    def start_threads(cls, color: str):
        Lidar.start_stop(1, color)

        Lidar.running.set()
        if Lidar._thread is None:
            Lidar._thread = Thread(target=Lidar._receive,
                                   args=(Lidar.running, ))
            # Lidar._thread = Thread(target=Lidar._receive, args=(Lidar.running, Lidar.opponent, Lidar.visualizer)) mislim da bi trebalo ovako ako ostane vizuelizacija ovde
            Lidar._thread.start()
        Lidar._logger.info("Lidar receiving thread started.")

    @classmethod
    def stop_threads(cls):
        Lidar.start_stop(0)

        Lidar.opponent.connection.close()
        Lidar.opponent.pc_socket.detach()
        Lidar.opponent.pc_socket.close()

        Lidar.running.clear()
        if Lidar._thread is not None and Lidar._thread.is_alive():
            Lidar._thread.join()
        Lidar._thread = None
        Lidar._logger.info("Lidar receiving thread stopped.")

    @classmethod
    def start_stop(cls, start_stop, color=None):
        send_queue = can_handler.msg_send_queues[IDs.SET_LIDAR.value]
        color_byte = 'y'.encode('utf_8') if color == "yellow" else 'b'.encode(
            'utf_8') if color == "blue" else '0'.encode('utf_8')
        lidar_msg = struct.pack('Bc', (start_stop & 0x01), color_byte)
        send_queue.append(lidar_msg)


if __name__ == "__main__":
    can_handler.start_threads()

    Lidar.start_threads()

    start_time = time.time()
    while (1):
        if time.time() - start_time > 600:
            break

    Lidar.stop_threads()

    can_handler.stop_threads()
