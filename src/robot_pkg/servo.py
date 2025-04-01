from enum import Enum
import struct, time
from threading import Thread, Event

from robot_pkg.main import log_handler, can_handler
from robot_pkg.can_controller import IDs

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
    send_queue = can_handler.msg_send_queues[IDs.SET_SERVO_POSITIONS.value]
    logger = log_handler.get_logger("servo")

    def __init__(self):
        self.id = 0
        self.position = 0
        self.speed = 0
        self._type = None
    
    def _execute(self):
        while not Servo.servo_in_position[self.id]:
            pass

        Servo.servo_list.extend([self.id, self.position, self.speed])
        Servo.servo_in_position[self.id] = False

    @classmethod
    def check_in_positions(cls):
        return all([in_position for servo, in_position in Servo.servo_in_position.items()])
    
    @classmethod
    def check_position(cls, id:int):
        queue = can_handler.msg_send_queues[IDs.GET_SERVO_POSITIONS.value]
        data = struct.pack('B', id)
        queue.append(data)

    @classmethod
    def send_positions(cls):
        if len(Servo.servo_list) > 0:
            # Servo.logger.debug(f"Sending: {Servo.servo_list}")
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

    @classmethod
    def RightVacuumLift(cls, position:int, speed:int=100):
        servo = cls()
        servo.id = ServoType.RIGHT_VACUUM_LIFT.value
        servo.position = position
        servo.speed = speed
        servo._type = ServoType.RIGHT_VACUUM_LIFT.name

        return servo

    @classmethod
    def RightVacuum(cls, position:int, speed:int=100):
        servo = cls()
        servo.id = ServoType.RIGHT_VACUUM.value
        servo.position = position
        servo.speed = speed
        servo._type = ServoType.RIGHT_VACUUM.name

        return servo

    @classmethod
    def LeftVacuumLift(cls, position:int, speed:int=100):
        servo = cls()
        servo.id = ServoType.LEFT_VACUUM_LIFT.value
        servo.position = position
        servo.speed = speed
        servo._type = ServoType.LEFT_VACUUM_LIFT.name

        return servo

    @classmethod
    def LeftVacuum(cls, position:int, speed:int=100):
        servo = cls()
        servo.id = ServoType.LEFT_VACUUM.value
        servo.position = position
        servo.speed = speed
        servo._type = ServoType.LEFT_VACUUM.name

        return servo

    @classmethod
    def RightGripLift(cls, position:int, speed:int=100):
        servo = cls()
        servo.id = ServoType.RIGHT_GRIP_LIFT.value
        servo.position = position
        servo.speed = speed
        servo._type = ServoType.RIGHT_GRIP_LIFT.name

        return servo

    @classmethod
    def LeftGripLift(cls, position:int, speed:int=100):
        servo = cls()
        servo.id = ServoType.LEFT_GRIP_LIFT.value
        servo.position = position
        servo.speed = speed        
        servo._type = ServoType.LEFT_GRIP_LIFT.name

        return servo

    @classmethod
    def CenterSwing(cls, position:int, speed:int=100):
        servo = cls()
        servo.id = ServoType.CENTER_SWING.value
        servo.position = position
        servo.speed = speed
        servo._type = ServoType.CENTER_SWING.name

        return servo

    @classmethod
    def CenterLift(cls, position:int, speed:int=100):
        servo = cls()
        servo.id = ServoType.CENTER_LIFT.value
        servo.position = position
        servo.speed = speed
        servo._type = ServoType.CENTER_LIFT.name

        return servo

    @classmethod
    def BackRightLift(cls, position:int, speed:int=100):
        servo = cls()
        servo.id = ServoType.BACK_RIGHT_LIFT.value
        servo.position = position
        servo.speed = speed
        servo._type = ServoType.BACK_RIGHT_LIFT.name

        return servo

    @classmethod
    def BackLeftLift(cls, position:int, speed:int=100):
        servo = cls()
        servo.id = ServoType.BACK_LEFT_LIFT.value
        servo.position = position
        servo.speed = speed
        servo._type = ServoType.BACK_LEFT_LIFT.name

        return servo

    