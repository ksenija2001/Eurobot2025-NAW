
# if __name__ != "__main__":
#     import sys, os

from robot_pkg.logger import LogHandler
from robot_pkg.can_controller import CanNetwork, IDs
import time
import can
import struct
import math

def main_func():
    log_handler = LogHandler()
    main_log, can_log = log_handler.get_loggers()
    main_log.info("Started code")
    can_handler = CanNetwork(channel='can0', interface='socketcan', max_queue_size=10, log=can_log)
    can_handler.start_threads()
    odom_msg:can.Message

    sent = False
    reset_msg = struct.pack('3f', 0.0, 0.0, 90*math.pi/180)
    can_handler.msg_send_queues[IDs.RESET_ODOM.value].append(reset_msg)

    time.sleep(2)
    try:
        start_time = time.time()
        while 1:
            if len(can_handler.msg_receive_queues[IDs.GET_ODOM.value]) > 0:
                odom_msg = can_handler.msg_receive_queues[IDs.GET_ODOM.value].pop()
                #print(odom_msg)
                
                [x, y, theta, left, right, trans, ang, gyr_ang] = struct.unpack('8f', odom_msg.data)
        

                print(f"ID:{hex(odom_msg.arbitration_id)}, x:{x:.2f}, y:{y:.2f}, theta:{theta*180/math.pi:.2f}, l_speed:{left:.2f}, r_speed:{right:.2f}, trans:{trans:.2f}, ang:{ang:.2f}")

            time.sleep(0.01)
            # if time.time() - start_time > 5 and not sent:
            #     sent = True
            #     can_handler.msg_send_queues[IDs.GET_POSITION.name].put((IDs.GET_POSITION, []))
            
    except KeyboardInterrupt:
        pass

    can_handler.stop_threads()


if __name__ == "__main__":
    main_func()
    