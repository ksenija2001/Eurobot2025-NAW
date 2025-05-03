from enum import Enum
import struct, time
from threading import Thread, Event

from robot_pkg.main import log_handler, can_handler
from robot_pkg.can_controller import IDs
from robot_pkg.move import Position
from robot_pkg.consts import Variables

class ActuatorType(Enum):
    PUMP = 4
    VALVE = 3

class SensorType(Enum):
    CINCH = 8
    FRONT_RIGHT = 3
    FRONT_CENTER_RIGHT = 2
    FRONT_CENTER_LEFT = 4
    FRONT_LEFT = 5
    BACK = 6


class I_O:
    _output_logger = log_handler.get_logger("outputs")
    _inputs_logger = log_handler.get_logger("inputs")
    _thread:Thread = None
    running:Event = Event()
    sensor_states:dict = {enum_item.value: False for enum_item in SensorType}
    send_queue = can_handler.msg_send_queues[IDs.SET_IO.value]

    def __init__(self):
        self.pin:int = 0
        self.on:bool = False
        self.send_pose = Position()
        self.sent = False
        self._type:str = None

    @classmethod
    def _receive(cls, running:Event):
        input_queue = can_handler.msg_receive_queues[IDs.GET_IO.value]
        while running.is_set():
            if len(input_queue) > 0:
                input_msg = input_queue.pop()

                [pin, state] = struct.unpack('2B', input_msg.data)

                if state and pin in I_O.sensor_states:
                    I_O.sensor_states[pin] = True
                    I_O._inputs_logger.info(f"Input {pin} enabled")
                elif pin in I_O.sensor_states:
                    I_O.sensor_states[pin] = False
                    I_O._inputs_logger.info(f"Input {pin} disabled")
                else:
                    I_O._inputs_logger.info(f"Input {pin} doesn't exist")
            
            # if any([state for pin, state in I_O.sensor_states.items() if pin in range(2, 6)]):
            #     Variables.front_detection.set()
            
            time.sleep(0.01)  # 10ms
    
    @classmethod
    def start_threads(cls):
        I_O.running.set()
        I_O.sensor_states[SensorType.CINCH.value] = 1
        if I_O._thread is None:
            I_O._thread = Thread(target=I_O._receive, args=(I_O.running, ))
            I_O._thread.start()
        I_O._inputs_logger.info("Input receiving thread started.")

    @classmethod
    def stop_threads(cls):
        I_O.running.clear()
        if I_O._thread is not None and I_O._thread.is_alive():
            I_O._thread.join()
        I_O._thread = None
        I_O._inputs_logger.info("Input receiving thread stopped.")
    
    @classmethod
    def Pump(cls, state:bool, send_pose:Position=Position()):
        pump = cls()
        pump.pin = ActuatorType.PUMP.value
        pump.state = state
        pump.send_pose = send_pose
        pump._type = ActuatorType.PUMP.name

        return pump

    @classmethod
    def Valve(cls, state:bool, send_pose:Position=Position()):
        valve = cls()
        valve.pin = ActuatorType.VALVE.value
        valve.state = state
        valve.send_pose = send_pose
        valve._type = ActuatorType.VALVE.name

        return valve
    
    def _execute(self):
        self.sent = True
        packed = [self.pin, self.state]
        data = struct.pack('2B', *packed)
        I_O.send_queue.append(data)