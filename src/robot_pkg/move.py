from enum import Enum
from threading import Thread, Event
import struct, math
import time

from robot_pkg.main import log_handler, can_handler
from robot_pkg.can_controller import IDs

class MoveType(Enum):
    RESET = 0
    RPM = 1
    SPEED = 2
    DISTANCE = 3
    TO_XY = 4
    ROTATE_TO = 5
    ROTATE_FOR = 6
    SPLINE = 7

class Position:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.theta = 0
    
    def reset(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta
    
    def __le__(self, other):
        return self.x <= other.x and self.y <= other.y
    
    def __ge__(self, other):
        return self.x >= other.x and self.y >= other.y


class Move:
    _logger = log_handler.get_logger("move")
    _odom_logger = log_handler.get_logger("odom")
    _thread:Thread = None
    running:Event = Event()
    move_done:Event = Event()
    pose = Position()
   
    def __init__(self):
        self.send_queue = None
        self.data:bytes 
        self._type:str = ""

    @classmethod
    def _receive(cls, running:Event):
        move_done_queue = can_handler.msg_receive_queues[IDs.GET_MOVE_DONE.value]
        odom_queue = can_handler.msg_receive_queues[IDs.GET_ODOM.value]
        while running.is_set():
            if len(move_done_queue) > 0:
                move_msg = move_done_queue.pop()

                success = struct.unpack('B', move_msg.data)

                if success:
                    Move.move_done.set()
                    Move._logger.info(f"Movement done")
                else:
                    # Movement unssuccsesful
                    pass
            
            if len(odom_queue) > 0:
                odom_msg = odom_queue.pop()

                [x, y, theta, left, right, trans, ang, gyr_ang] = struct.unpack('8f', odom_msg.data)
                Move.pose.x = x
                Move.pose.y = y
                Move.pose.theta = theta

                Move._odom_logger.debug(f"x:{x:4.2f}, y:{y:4.2f}, theta:{theta*180/math.pi:4.2f}, l_speed:{left:4.2f}, r_speed:{right:4.2f}, trans:{trans:4.2f}, ang:{ang:4.2f}")
            
            time.sleep(0.01)  # 10ms
    
    @classmethod
    def start_threads(cls):
        Move.running.set()
        Move.move_done.set()
        if Move._thread is None:
            Move._thread = Thread(target=Move._receive, args=(Move.running, ))
            Move._thread.start()
        Move._logger.info("Move done and Odometry receiving thread started.")

    @classmethod
    def stop_threads(cls):
        Move.running.clear()
        if Move._thread is not None and Move._thread.is_alive():
            Move._thread.join()
        Move._thread = None
        Move._logger.info("Move done and Odometry receiving thread stopped.")

    @classmethod
    def ResetOdom(cls, x:float, y:float, theta:float):
        '''
            Sets current odometry to (x,y,theta).
        '''
        move = cls()
        move.data = struct.pack('3f', x, y, theta)
        move.send_queue = can_handler.msg_send_queues[IDs.RESET_ODOM.value]
        move._type = MoveType.RESET.name

        return move

    @classmethod
    def RPM(cls, left_rpm:int, right_rpm:int):
        '''
            Sets target speed[RPM] for both motors.
        '''
        move = cls()
        move.data = struct.pack('2i', left_rpm, right_rpm)
        move.send_queue = can_handler.msg_send_queues[IDs.SET_MOTOR_RPM.value]
        move._type = MoveType.RPM.name

        return move

    @classmethod
    def Speed(cls, left_speed:int, right_speed:int):
        '''
            Sets target speed[mm/s] for both motors.
        '''
        move = cls()
        move.data = struct.pack('2i', left_speed, right_speed)
        move.send_queue = can_handler.msg_send_queues[IDs.SET_MOTOR_SPEED.value]
        move._type = MoveType.SPEED.name

        return move

    @classmethod
    def Distance(cls, p:float, v:float, a:float):
        '''
            Starts relative movement of distance[mm] from current robot position 
            with respect to velocity and acceleration limits.
        '''
        move = cls()
        move.data = struct.pack('3f', p, v, a)
        move.send_queue = can_handler.msg_send_queues[IDs.SET_DISTANCE.value]
        move._type = MoveType.DISTANCE.name

        return move

    @classmethod
    def To(cls, x_coor:float, y_coor:float, direction:bool, v:float, a:float, w:float, alpha:float):
        '''
            Starts absolute movement to (x,y) coordinate of table with respect to 
            velocity and acceleration limits.
        '''
        move = cls()
        move.data = struct.pack('ffiffff', x_coor, y_coor, direction, v, a, w, alpha)
        move.send_queue = can_handler.msg_send_queues[IDs.SET_XY.value]
        move._type = MoveType.TO_XY.name

        return move

    @classmethod
    def Rotate(cls, theta:float, w:float, alpha:float):
        '''
            Starts relative rotation of theta[rad] from current orientation of robot 
            with respect to angular velocity and acceleration limits.
        '''
        move = cls()
        move.data = struct.pack('3f', theta, w, alpha)
        move.send_queue = can_handler.msg_send_queues[IDs.SET_ROTATION_FOR.value]
        move._type = MoveType.ROTATE_FOR.name

        return move

    @classmethod
    def RotateTo(cls, theta:float, w:float, alpha:float):
        '''
            Starts absolute rotation to theta[rad] with respect to 
            angular velocity and acceleration limits.
        '''
        move = cls()
        move.data = struct.pack('3f', theta, w, alpha)
        move.send_queue = can_handler.msg_send_queues[IDs.SET_ROTATION_TO.value]
        move._type = MoveType.ROTATE_TO.name

        return move

    @classmethod
    def Spline(cls, x:list, y:list, theta:list, speed:int, size:int):
        '''
            Points (x,y,theta) define a curve the robot will follow with designated speed.
        '''
        move = cls()
        move.data = struct.pack('>B'+'f'*3*size+'H', *[el for tup in list(zip(x, y, theta)) for el in tup])
        move.send_queue = can_handler.msg_send_queues[IDs.SET_SPLINE.value]
        move._type = MoveType.SPLINE.name

        return move
    
    def _execute(self):
        # Waits for previous move command to complete
        Move.move_done.wait()

        self.send_queue.append(self.data)

        Move.move_done.clear()