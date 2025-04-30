from enum import Enum
import struct, time
from threading import Thread, Event

from robot_pkg.main import log_handler, can_handler
from robot_pkg.can_controller import IDs
from robot_pkg.move import Position

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
    FRONT_RIGHT_GRIPPER = 11        # 0 - open, 180 - closed
    FRONT_CENTER_RIGHT_GRIPPER = 12 # 0 - closed, 180 - open
    FRONT_CENTER_LEFT_GRIPPER = 13  # 0 - open, 180 - closed
    FRONT_LEFT_GRIPPER = 14         # 0 - closed, 180 - open
    BACK_RIGHT_GRIPPER = 15         
    BACK_CENTER_RIGHT_GRIPPER = 16  # 0 - closed, 180 - open
    BACK_CENTER_LEFT_GRIPPER = 17   # 0 - open, 180 - closed
    BACK_LEFT_GRIPPER = 18


class Servo:
    servo_list:list[int] = []
    # rc_servo_list:list[int] = []
    servo_thread:Thread = None
    running:Event = Event()
    servo_in_position:dict = {enum_item.value: True for enum_item in ServoType}
    send_queue = can_handler.msg_send_queues[IDs.SET_SERVO_POSITIONS.value]
    rc_send_queue = can_handler.msg_send_queues[IDs.SET_RC_SERVO_POSITIONS.value]
    servo_positions:dict = {enum_item.value: 0 for enum_item in ServoType}
    
    logger = log_handler.get_logger("servo")

    def __init__(self):
        self.id = 0
        self.position = 0
        self.speed = 0
        self.executed = False
        self.activate_pose = Position()
        self._type = None
    
    def _execute(self):
        while self.id <= 10 and not Servo.servo_in_position[self.id]:
            pass
        
        self.executed = True
        if self.id <= 10:
            Servo.servo_list.extend([self.id, self.position, self.speed])
            Servo.servo_in_position[self.id] = False
        else:
            data = [self.id, self.position]
            servo_msg = struct.pack('2B', *data)
            Servo.logger.debug(f"Sending: {data}")

            Servo.rc_send_queue.append(servo_msg)
            Servo.servo_in_position[self.id] = False

    def check_in_position(self):
        return Servo.servo_in_position[self.id]

    @classmethod
    def check_in_positions(cls):
        return all([in_position for servo, in_position in Servo.servo_in_position.items() if servo <= 10])
    
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
        error_queue = can_handler.msg_receive_queues[IDs.GET_SERVO_ERROR.value]
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

                Servo.servo_positions[id] = angle
                Servo.logger.info(f"Servo {id} position: {angle}")
            
            if len(error_queue) > 0:
                servo_msg = error_queue.pop()

                [id, error] = struct.unpack('2B', servo_msg.data)
                Servo.logger.info(f"Servo {id} error: {error}")

            time.sleep(0.001)  # 1ms
    
    @classmethod
    def start_threads(cls):
        Servo.running.set()
        msg = struct.pack('B', 1)
        can_handler.msg_send_queues[IDs.SET_SERVO_TORQUE.value].append(msg)
        if Servo.servo_thread is None:
            Servo.servo_thread = Thread(target=Servo._receive, args=(Servo.running, ))
            Servo.servo_thread.start()
        Servo.logger.info("Servo receiving thread started.")

    @classmethod
    def stop_threads(cls):
        Servo.running.clear()
        msg = struct.pack('B', 0)
        can_handler.msg_send_queues[IDs.SET_SERVO_TORQUE.value].append(msg)

        if Servo.servo_thread is not None and Servo.servo_thread.is_alive():
            Servo.servo_thread.join()
        Servo.servo_thread = None
        Servo.logger.info("Servo receiving thread stopped.")

    @classmethod
    def FrontVacuumLift(cls, position:int, speed:int=100, activate_pose=Position()):
        servo1 = cls()
        servo2 = cls()

        servo1.id = ServoType.RIGHT_VACUUM_LIFT.value
        servo1.position = 300 - position
        servo1.speed = speed
        servo1.activate_pose = activate_pose
        servo1._type = ServoType.RIGHT_VACUUM_LIFT.name

        servo2.id = ServoType.LEFT_VACUUM_LIFT.value
        servo2.position = position
        servo2.speed = speed
        servo2.activate_pose = activate_pose
        servo2._type = ServoType.LEFT_VACUUM_LIFT.name

        return servo1, servo2
    
    @classmethod
    def FrontVacuum(cls, position:int, speed:int=100, activate_pose=Position()):
        servo1 = cls()
        servo2 = cls()

        servo1.id = ServoType.RIGHT_VACUUM.value
        servo1.position = 300 - position
        servo1.speed = speed
        servo1.activate_pose = activate_pose
        servo1._type = ServoType.RIGHT_VACUUM.name

        servo2.id = ServoType.LEFT_VACUUM.value
        servo2.position = position
        servo2.speed = speed
        servo2.activate_pose = activate_pose
        servo2._type = ServoType.LEFT_VACUUM.name

        return servo1, servo2

    @classmethod
    def FrontGripLift(cls, position:int, speed:int=100, activate_pose=Position()):
        servo1 = cls()
        servo2 = cls()

        servo1.id = ServoType.RIGHT_GRIP_LIFT.value
        servo1.position = 300 - position
        servo1.speed = speed
        servo1.activate_pose = activate_pose
        servo1._type = ServoType.RIGHT_GRIP_LIFT.name

        servo2.id = ServoType.LEFT_GRIP_LIFT.value
        servo2.position = position
        servo2.speed = speed
        servo2.activate_pose = activate_pose
        servo2._type = ServoType.LEFT_GRIP_LIFT.name

        return servo1, servo2

    @classmethod
    def CenterSwing(cls, position:int, speed:int=100, activate_pose=Position()):
        servo = cls()
        servo.id = ServoType.CENTER_SWING.value
        servo.position = position
        servo.speed = speed
        servo.activate_pose = activate_pose
        servo._type = ServoType.CENTER_SWING.name

        return servo

    @classmethod
    def CenterLift(cls, position:int, speed:int=100, activate_pose=Position()):
        servo = cls()
        servo.id = ServoType.CENTER_LIFT.value
        servo.position = 300 - position
        servo.speed = speed
        servo.activate_pose = activate_pose
        servo._type = ServoType.CENTER_LIFT.name

        return servo

    @classmethod
    def BackLift(cls, position:int, speed:int=50, activate_pose=Position()):
        servo1 = cls()
        servo2 = cls()

        servo1.id = ServoType.BACK_RIGHT_LIFT.value
        servo1.position = 300 - position
        servo1.speed = speed
        servo1.activate_pose = activate_pose
        servo1._type = ServoType.BACK_RIGHT_LIFT.name

        servo2.id = ServoType.BACK_LEFT_LIFT.value
        servo2.position = position
        servo2.speed = speed
        servo2.activate_pose = activate_pose
        servo2._type = ServoType.BACK_LEFT_LIFT.name

        return servo1, servo2
    
    @classmethod
    def FrontSideGrip(cls, left_position:int, right_position:int=0, activate_pose=Position()):
        servo1 = cls()
        servo2 = cls()

        servo1.id = ServoType.FRONT_RIGHT_GRIPPER.value
        servo1.position = right_position
        servo1.activate_pose = activate_pose
        servo1._type = ServoType.FRONT_RIGHT_GRIPPER.name

        servo2.id = ServoType.FRONT_LEFT_GRIPPER.value
        servo2.position = left_position
        servo2.activate_pose = activate_pose
        servo2._type = ServoType.FRONT_LEFT_GRIPPER.name

        return servo1, servo2
    
    @classmethod
    def FrontCenterGrip(cls, left_position:int, right_position:int=0, activate_pose=Position()):
        servo1 = cls()
        servo2 = cls()

        servo1.id = ServoType.FRONT_CENTER_RIGHT_GRIPPER.value
        servo1.position = right_position
        servo1.activate_pose = activate_pose
        servo1._type = ServoType.FRONT_CENTER_RIGHT_GRIPPER.name

        servo2.id = ServoType.FRONT_CENTER_LEFT_GRIPPER.value
        servo2.position = left_position
        servo2.activate_pose = activate_pose
        servo2._type = ServoType.FRONT_CENTER_LEFT_GRIPPER.name

        return servo1, servo2

    @classmethod
    def BackSideGrip(cls, left_position:int, right_position:int=0, activate_pose=Position()):
        servo1 = cls()
        servo2 = cls()

        servo1.id = ServoType.BACK_RIGHT_GRIPPER.value
        servo1.position =  right_position
        servo1.activate_pose = activate_pose
        servo1._type = ServoType.BACK_RIGHT_GRIPPER.name

        servo2.id = ServoType.BACK_LEFT_GRIPPER.value
        servo2.position = left_position
        servo2.activate_pose = activate_pose
        servo2._type = ServoType.BACK_LEFT_GRIPPER.name

        return servo1, servo2
    
    @classmethod
    def BackCenterGrip(cls, left_position:int, right_position:int=0, activate_pose=Position()):
        servo1 = cls()
        servo2 = cls()

        servo1.id = ServoType.BACK_CENTER_RIGHT_GRIPPER.value
        servo1.position = right_position
        servo1.activate_pose = activate_pose
        servo1._type = ServoType.BACK_CENTER_RIGHT_GRIPPER.name

        servo2.id = ServoType.BACK_CENTER_LEFT_GRIPPER.value
        servo2.position = left_position
        servo2.activate_pose = activate_pose
        servo2._type = ServoType.BACK_CENTER_LEFT_GRIPPER.name

        return servo1, servo2


    