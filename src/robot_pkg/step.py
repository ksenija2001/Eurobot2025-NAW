import struct, time
from threading import Thread, Event
from collections import deque
from enum import Enum

from robot_pkg.main import can_handler, log_handler
from robot_pkg.consts import IDs

class ServoType(Enum):
    RIGHT_VACUUM_LIFT = 1
    RIGHT_VACUUM = 2
    LEFT_VACUUM_LIFT = 3 
    LEFT_VACUUM = 4    
    RIGHT_GRIP_LIFT = 5  
    LEFT_GRIP_LIFT = 6  
    CENTER_SWING = 7   
    CENTER_LIFT = 8    
    BACK_RIGHT_LIFT = 9  
    BACK_LEFT_LIFT = 10 

class Servo:
    servo_list:list[int] = []
    servo_thread:Thread = None
    running:Event = Event()
    servo_in_position:dict = {enum_item.value: True for enum_item in ServoType}
    send_queue:deque = can_handler.msg_send_queues[IDs.SET_SERVO_POSITIONS.value]
    logger = log_handler.get_logger("servo")

    def __init__(self):
        self.id = 0
        self.position = 0
        self.speed = 0

    @classmethod
    def RightVacuumLift(cls, position:int, speed:int):
        servo = cls()
        servo.id = ServoType.RIGHT_VACUUM_LIFT.value
        servo.position = position
        servo.speed = speed

        return servo

    @classmethod
    def RightVacuum(cls, position:int, speed:int):
        servo = cls()
        servo.id = ServoType.RIGHT_VACUUM.value
        servo.position = position
        servo.speed = speed

        return servo

    @classmethod
    def LeftVacuumLift(cls, position:int, speed:int):
        servo = cls()
        servo.id = ServoType.LEFT_VACUUM_LIFT.value
        servo.position = position
        servo.speed = speed

        return servo

    @classmethod
    def LeftVacuum(cls, position:int, speed:int):
        servo = cls()
        servo.id = ServoType.LEFT_VACUUM.value
        servo.position = position
        servo.speed = speed

        return servo

    @classmethod
    def RightGripLift(cls, position:int, speed:int):
        servo = cls()
        servo.id = ServoType.RIGHT_GRIP_LIFT.value
        servo.position = position
        servo.speed = speed

        return servo

    @classmethod
    def LeftGripLift(cls, position:int, speed:int):
        servo = cls()
        servo.id = ServoType.LEFT_GRIP_LIFT.value
        servo.position = position
        servo.speed = speed

        return servo

    @classmethod
    def CenterSwing(cls, position:int, speed:int):
        servo = cls()
        servo.id = ServoType.CENTER_SWING.value
        servo.position = position
        servo.speed = speed

        return servo

    @classmethod
    def CenterLift(cls, position:int, speed:int):
        servo = cls()
        servo.id = ServoType.CENTER_LIFT.value
        servo.position = position
        servo.speed = speed

        return servo

    @classmethod
    def BackRightLift(cls, position:int, speed:int):
        servo = cls()
        servo.id = ServoType.BACK_RIGHT_LIFT.value
        servo.position = position
        servo.speed = speed

        return servo

    @classmethod
    def BackLeftLift(cls, position:int, speed:int):
        servo = cls()
        servo.id = ServoType.BACK_LEFT_LIFT.value
        servo.position = position
        servo.speed = speed

        return servo

    def _execute(self):
        while not Servo.servo_in_position[self.id]:
            pass

        Servo.servo_list.extend([self.id, self.position, self.speed])
        Servo.servo_in_position[self.id] = False
    
    @classmethod
    def check_position(cls, id:int):
        queue = can_handler.msg_send_queues[IDs.GET_SERVO_POSITIONS.value]
        data = struct.pack('B', id)
        queue.append(data)

    @classmethod
    def send_positions(cls):
        if len(Servo.servo_list) > 0:
            Servo.logger.debug(f"Sending: {Servo.servo_list}")
            size = len(Servo.servo_list)//3
            Servo.servo_list.insert(0, size)

            fmt = ">B" + "BHB"*size 
            servo_msg = struct.pack(fmt, *Servo.servo_list)
            Servo.send_queue.append(servo_msg)

            Servo.servo_list.clear()

    @classmethod
    def _receive(cls, running:Event):
        in_position_queue = can_handler.msg_receive_queues[IDs.GET_SERVO_IN_POSITION.value]
        positions_queue = can_handler.msg_receive_queues[IDs.GET_SERVO_POSITIONS.value]
        while running.is_set():
            if len(in_position_queue) > 0:
                servo_msg = in_position_queue.pop()

                [id, success] = struct.unpack('2B', servo_msg.data)

                if success:
                    Servo.servo_in_position[id] = True
                    Servo.logger.info(f"Servo {id} in position")
                else:
                    # Servo did not reach position
                    pass
            
            if len(positions_queue) > 0:
                servo_msg = positions_queue.pop()

                [id, angle_high, angle_low] = struct.unpack('3B', servo_msg.data)
                angle = int.from_bytes([angle_high, angle_low])
        
                Servo.logger.info(f"Servo {id} position: {angle}")

            time.sleep(0.01)  # 10ms
    
    @classmethod
    def start_threads(cls):
        Servo.running.set()
        if Servo.servo_thread is None:
            Servo.servo_thread = Thread(target=Servo._receive, args=(Servo.running, ))
            Servo.servo_thread.start()
        Servo.logger.info("Servo receiving thread started.")

    @classmethod
    def stop_threads(cls):
        Servo.running.clear()
        if Servo.servo_thread is not None and Servo.servo_thread.is_alive():
            Servo.servo_thread.join()
        Servo.servo_thread = None
        Servo.logger.info("Servo receiving thread stopped.")

class Move:
    ack_queue = can_handler.msg_receive_queues[IDs.GET_MOVE_DONE.value]

    def __init__(self):
        self.send_queue:deque = None
        self.data:bytes 

    @classmethod
    def RPM(cls, left_rpm:int, right_rpm:int):
        '''
            Sets target speed[RPM] for both motors.
        '''
        move = cls()
        move.data = struct.pack('2i', left_rpm, right_rpm)
        move.send_queue = can_handler.msg_send_queues[IDs.SET_MOTOR_RPM.value]

        return move

    @classmethod
    def Speed(cls, left_speed:int, right_speed:int):
        '''
            Sets target speed[mm/s] for both motors.
        '''
        move = cls()
        move.data = struct.pack('2i', left_speed, right_speed)
        move.send_queue = can_handler.msg_send_queues[IDs.SET_MOTOR_SPEED.value]

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

        return move
    
    def _execute(self):
        self.send_queue.append(self.data)


if __name__ == "__main__":
    Servo.start_threads()

    servo1 = Servo.RightVacuumLift(100, 10)
    servo2 = Servo.RightVacuum(150, 10)

    servo1._execute()
    servo2._execute()

    Servo.send_positions()

    time.sleep(5)

    servo1 = Servo.RightVacuumLift(150, 10)
    servo2 = Servo.RightVacuum(100, 10)

    servo1._execute()
    servo2._execute()

    Servo.send_positions()

    # servo1._check_position()
    
    time.sleep(5)


    Servo.stop_threads()







    
