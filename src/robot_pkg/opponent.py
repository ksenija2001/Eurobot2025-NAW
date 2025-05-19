import socket, time, struct
from robot_pkg.play_elements import Area, MaterialStack

from robot_pkg.main import log_handler

class Position:
    def __init__(self, x: float = 0, y: float = 0, theta: float = 0, speed: float = 0):
        self.x = x
        self.y = y
        self.theta = theta
        self.speed = speed
        self.left_inc = 0
        self.right_inc = 0

    def reset(self, x, y, theta, speed=0):
        self.x = x
        self.y = y
        self.theta = theta
        self.speed = speed

    def __repr__(self):
        return f"{self.x}, {self.y}"

class Opponent:
    pose = Position(0, 0, 0)
    _logger = log_handler.get_logger("opponent")
    last_area = None
    area_entry_time = None
    last_stack = None
    stack_entry_time = None
    pc_socket = None

    @classmethod
    def _setup_connection(cls):
        try:
            cls.pc_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            cls.pc_socket.setsockopt(
                socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            cls.pc_socket.connect(('10.166.197.67', 9999))

            cls._logger.info("Computer connected")
        except Exception as e:
            print(f"Socket exception: {e}")
            cls.pc_socket = None
            pass

    @classmethod
    def _send_opponent_info(cls, data):
        if cls.pc_socket is not None:
            try:
                if cls.pc_socket is not None:
                    cls.pc_socket.send(data)
            except Exception as e:
                print(e)
                pass

    @classmethod
    def update_position(cls, x: float, y: float, theta: float, speed: float):
        """
        Update opponent's current position and check if it is in any area or stack
        """
        cls.pose.reset(x, y, theta, speed)

        cls._check_area_occupation()
        cls._check_stack_occupation()

    @classmethod
    def _check_area_occupation(cls):
        """
        Check if opponent is in any defined area and mark as visited if stays for more than 2 seconds
        """

        current_area = None
        area_name = None

        for name, area in Area.get_all_areas():
            if (area.x - 225 <= cls.pose.x <= area.x + 225 and
                    area.y - 225 <= cls.pose.y <= area.y + 225):

                current_area = area
                area_name = name
                break

        if current_area != cls.last_area:
            # Opponent moved to a new area or left the previous one
            cls.last_area = current_area
            cls.area_entry_time = time.time() if current_area else None
        elif current_area is not None and (time.time() - cls.area_entry_time > 2):
            # Opponent has been in this area for more than 2 seconds
            if not current_area.visited:
                current_area.visited = True

                s = bytes(area_name, 'utf-8')

                data = ['A'.encode('utf-8'), len(s)]
                op_data = struct.pack('cB', *data) + s
                print(f"Area data: {op_data}")
                cls._send_opponent_info(op_data)

                cls._logger.debug(
                    f"Opponent marked area {current_area} as visited")

    @classmethod
    def _check_stack_occupation(cls):
        """
        Check if opponent is near any stack and mark as visited if stays for more than two seconds
        """
        current_stack: MaterialStack = None
        stack_name = None

        for name, stack in MaterialStack.get_all_unvisited_stacks():
            # 200 mm radius
            if stack.distance_to(cls.pose.x, cls.pose.y) < 200:
                current_stack = stack
                stack_name = name
                break

        if current_stack != cls.last_stack:
            # Opponent moved to a new stack or left the previous one
            cls.last_stack = current_stack
            cls.stack_entry_time = time.time() if current_stack else None
        elif current_stack is not None and (time.time() - cls.stack_entry_time > 2):
            # Opponent has been near this stack for more than 2 seconds
            if not current_stack.visited:
                current_stack.visited = True

                s = bytes(stack_name, 'utf-8')

                data = ['S'.encode('utf-8'), len(s)]
                op_data = struct.pack('cB', *data) + s
                print(f"Stack data: {op_data}")
                cls._send_opponent_info(op_data)
                # You might want to log this event
                cls._logger.debug(
                    f"Opponent marked stack {current_stack} as visited")

