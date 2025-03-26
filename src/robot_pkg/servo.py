from enum import Enum
import struct, time
from threading import Thread, Event

from robot_pkg.main import log_handler, can_handler
from robot_pkg.can_controller import IDs
from robot_pkg.step import Servo

# class Servo:
#     def __init__(self, id:int, top, middle, bottom):
#         self.id = id

#         self.TOP = top
#         self.MIDDLE = middle
#         self.BOTTOM = bottom
  
#         self.in_position = False

class ServoTypes(Enum):
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

# AXServos = {
#     ServoNames.RIGHT_VACUUM_LIFT : Servo(id=1,  top=0,   middle=0,   bottom=300),
#     ServoNames.RIGHT_VACUUM      : Servo(id=2,  top=240, middle=150, bottom=60),
#     ServoNames.LEFT_VACUUM_LIFT  : Servo(id=3,  top=300, middle=0,   bottom=0),
#     ServoNames.LEFT_VACUUM       : Servo(id=4,  top=60,  middle=150, bottom=240),
#     ServoNames.RIGHT_GRIP_LIFT   : Servo(id=5,  top=0,   middle=0,   bottom=300),
#     ServoNames.LEFT_GRIP_LIFT    : Servo(id=6,  top=300, middle=0,   bottom=0),
#     ServoNames.CENTER_SWING      : Servo(id=7,  top=240, middle=0,   bottom=150),
#     ServoNames.CENTER_LIFT       : Servo(id=8,  top=0,   middle=0,   bottom=242),
#     ServoNames.BACK_RIGHT_LIFT   : Servo(id=9,  top=240, middle=0,   bottom=150),
#     ServoNames.BACK_LEFT_LIFT    : Servo(id=10, top=0,   middle=0,   bottom=242)
# }

class ServoHandler:
    def __init__(self):
        self.log = log_handler.get_logger("servo")
        self.queue = can_handler.msg_receive_queues[IDs.GET_SERVO_IN_POSITION.value]
        self.position_queue = can_handler.msg_receive_queues[IDs.GET_SERVO_POSITIONS.value]
        self.send_queue = can_handler.msg_send_queues[IDs.SET_SERVO_POSITIONS.value]

        self.running = False
        self._servo_thread = Thread(target=self.receive)
        self._servo_list = []
        self.servo_in_position = {enum_item.value: True for enum_item in ServoTypes}

    def start(self):
        self.running = True
        self._servo_thread.start()

        self.log.info(f"Started ServoHandler")

    def stop(self):
        self.running = False
        self._servo_thread.join()

        self.log.info(f"Stopped ServoHandler")

    def add_angle(self, id:int, angle:int, speed:int):
        self._servo_list.append(id)
        self._servo_list.append(angle)
        self._servo_list.append(speed)

        self.servo_in_position[id] = False

    def sync_write(self):
        size = len(self._servo_list)/3
        self._servo_list.insert(0, size)
        fmt = ">B" + "BHB"*size 
        servo_msg = struct.pack(fmt, *self._servo_list)
        self.send_queue.append(servo_msg)
        self._servo_list.clear()

    def receive(self):
        while self.running:
            if len(self.queue) > 0:
                servo_msg = self.queue.pop()

                [id, success] = struct.unpack('2B', servo_msg.data)

                if success:
                    Servo.servo_in_position[id] = True
                else:
                    # Servo did not reach position
                    pass

                self.log.debug(f"Servo {id}: {success}")
            
            if len(self.position_queue) > 0:
                servo_msg = self.position_queue.pop()

                [id, angle_high, angle_low] = struct.unpack('3I', servo_msg.data)

                self.log.debug(f"{id}: {(int)(angle_high << 8) & angle_low}")

            time.sleep(0.01)  # 10ms


if __name__ == "__main__":
    pass

