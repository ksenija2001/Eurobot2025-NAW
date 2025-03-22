from enum import EnumDict
import struct, time
from threading import Thread

from robot_pkg.main import log_handler, can_handler
from robot_pkg.can_controller import IDs

class Servo:
    def __init__(self, id:int, top, middle, bottom):
        self.id = id

        self.TOP = top
        self.MIDDLE = middle
        self.BOTTOM = bottom
  
        self.in_position = False

class Servos(EnumDict):
    RIGHT_VACUUM_LIFT = Servo(id=1,  top=0,   middle=0,   bottom=300),
    RIGHT_VACUUM      = Servo(id=2,  top=240, middle=150, bottom=60),
    LEFT_VACUUM_LIFT  = Servo(id=3,  top=300, middle=0,   bottom=0),
    LEFT_VACUUM       = Servo(id=4,  top=60,  middle=150, bottom=240),
    RIGHT_GRIP_LIFT   = Servo(id=5,  top=0,   middle=0,   bottom=300),
    LEFT_GRIP_LIFT    = Servo(id=6,  top=300, middle=0,   bottom=0),
    CENTER_SWING      = Servo(id=7,  top=240, middle=0,   bottom=150),
    CENTER_LIFT       = Servo(id=8,  top=0,   middle=0,   bottom=242),
    BACK_RIGHT_LIFT   = Servo(id=9,  top=240, middle=0,   bottom=150),
    BACK_LEFT_LIFT    = Servo(id=10, top=0,   middle=0,   bottom=242)

class ServoHandler:
    def __init__(self):
        self.log = log_handler.get_logger("servo")
        self.queue = can_handler.msg_receive_queues[IDs.GET_SERVO_IN_POSITION.value],
        self.send_queue = can_handler.msg_send_queues[IDs.SET_SERVO_POSITIONS.value],

        self.running = False
        self._servo_thread = Thread(target=self.receive)

    def start(self):
        self.running = True
        self._servo_thread.start()

        self.log.info(f"Started ServoHandler")

    def stop(self):
        self.running = False
        self._servo_thread.join()

        self.log.info(f"Stopped ServoHandler")

    def set_angles(self, ids:list[int], angles:list[int], speeds:list[int]):
        size = len(ids)

        if size != len(angles) or size != len(speeds):
            print("Wrong number of parameters!")
            raise Exception

        servo_msg = struct.pack(size*3 + 'I', ids, angles, speeds)
        self.send_queue.appends(servo_msg)

    def receive(self):
        while self.running:
            if len(self.queue) > 0:
                servo_msg = self.queue.pop()

                [id, success] = struct.unpack('2I', servo_msg.data)

                servo = [servo for servo in Servos.values() if servo.id == id][0]

                if success:
                    servo.in_position = True

                self.log.debug(f"{id}: {success}")
            
            time.sleep(0.01)  # 10ms


if __name__ == "__main__":
    pass

