import can
from enum import Enum
from threading import Thread, Event
from queue import Queue, Empty
from collections import deque
import struct
import logging
import time

class IDs(Enum):
    RESET_ODOM  = 0x4F0
    ODOM_CONFIG = 0x4F1

    GET_ODOM = 0x4FF

    SET_MOTOR_SPEED = 0x4D0
    SET_MOTOR_RPM   = 0x4D1
    SET_DISTANCE    = 0x4D2
    SET_XY          = 0x4D3
    SET_ROTATION_FOR = 0x4D4
    SET_ROTATION_TO  = 0x4D5

    GET_MOTOR_SPEED = 0x4DF
    GET_MOVE_DONE   = 0x4DE

    SET_SERVO_POSITIONS = 0x530

    GET_PAUSE  = 0x50F


# class CanGateway:
#     '''
#         Handles packet parsing and message priorities
#     '''

#     def init(self, msg_types:dict[IDs, deque]):
#         self.running = False
#         pass

    

#     def start_checking(self):
#         self.running = True

#     def check_queue(self, queue:deque):
#         while self.running:
#             msg = queue.get()
#             self.parse(msg.arbitration_id)
            
#     def parse(self, msg_id):
#         if msg_id == IDs.GET_POSITION:
#             pass
#         elif msg_id == IDs.GET_SERVO_DONE:
#             pass
#         elif msg_id == IDs.GET_SERVO_POSITION:
#             pass

#     def wait_for(self, msg_ids:list) -> bool:
#         pass

                




class CanNetwork:
    '''
        Handles all traffic on CAN network - sending and receiving of packets
    '''

    def __init__(self, channel, interface, max_queue_size, log: logging.Logger):
        can.rc['interface'] = interface
        can.rc['channel'] = channel
        can.rc['fd'] = True

        self.bus = can.Bus()

        self.logger = log

        self.msg_receive_queues = {}
        self.msg_send_queues = {}
        self.max_queue_size = max_queue_size

        self._recv_thread = Thread(target=self.receive)
        self._send_thread = Thread(target=self.send)
        self.running = False

        self.logger.info("CAN Handler initialised.")

    def init_queues(self, max_queue_size=0):
        for msg_type in IDs:
            self.msg_receive_queues[msg_type.value] = deque(maxlen=max_queue_size)
            self.msg_send_queues[msg_type.value] = deque(maxlen=max_queue_size)

    def request_msg(self, msg_id:IDs, size):
        '''
            msg_id: ID of requested data,
            size  : expected size of the received data
        '''

        msg = can.Message(arbitration_id=msg_id.value, dlc=size, data=[], is_extended_id=False, is_fd=True, is_remote_frame=True)

        try:
            # with can.Bus() as bus:
            #     bus.send(msg)
            print(f"Message sent") # on {bus.channel_info}")
        except can.CanError:
            print("[ERROR] Message NOT sent")
    
    def start_threads(self):
        self.init_queues(self.max_queue_size)
        

        self.running = True
        self._recv_thread.start()
        self.logger.info(f"CAN receiving thread started.")

        self._send_thread.start()
        self.logger.info(f"CAN sending thread started.")
    
    def stop_threads(self):
        self.running = False
        self._recv_thread.join()
        self._send_thread.join()
        self.logger.info(f"Threads stopped.")

    def receive(self):
        while self.running:
            try:
                msg = self.bus.recv(timeout=0.1)                  # blocks until a message is ready
                if msg is not None:
                    msg_id = msg.arbitration_id
                    self.msg_receive_queues[msg_id].append(msg)  # stores received message in appropriate queue
            
                    self.logger.debug(f"Message {IDs(msg_id).name} put into queue.")
                    # self.logger.debug(f"Queue length: {len(self.msg_receive_queues[msg_id])}")
            except can.CanError as e:
                self.logger.warning(f"Message NOT received correctly: {e}")
            
            time.sleep(0.0005) # 0.5ms

    def send(self):
        key = None
        while self.running:
            for key, queue in self.msg_send_queues.items():   #   priorities are determined by the order they were listed in IDs
                try:
                    
                    if len(queue) > 0:
                        data = queue.pop()
                        msg = can.Message(arbitration_id=key, data=data, is_extended_id=False, is_fd=True)

                        self.bus.send(msg)
                        self.logger.debug(f"Message {IDs(key).name} sent")
                
                except Empty:
                    pass
                except can.CanError as e:
                        self.logger.warning(f"Message {IDs(key).name} NOT sent: {e}")

            time.sleep(0.001) # 1ms
    
    def __del__(self):
        self.bus.shutdown()
                    
                
                


      