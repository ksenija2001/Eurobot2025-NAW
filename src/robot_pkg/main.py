
from robot_pkg.logger import LogHandler
from robot_pkg.can_controller import CanNetwork, IDs
from robot_pkg.odometry import OdometryHandler, Odometry
from threading import Event
import time
import can
import struct
import math

# Global access to can_handler for steps
can_handler:CanNetwork
paused:Event

def main_func():
    log_handler = LogHandler()
    main_log = log_handler.get_logger("main")
    can_log = log_handler.get_logger("can")
    odom_log = log_handler.get_logger("odom")

    main_log.info("Started code")
    can_handler = CanNetwork(channel='can0', interface='socketcan', max_queue_size=10, log=can_log)
    can_handler.start_threads()

    odom = OdometryHandler(can_handler.msg_receive_queues[IDs.GET_ODOM.value],
                           can_handler.msg_send_queues[IDs.RESET_ODOM.value],
                           odom_log,
                           Odometry(0.0, 0.0, 90*math.pi/180))
    
    pause_queue = can_handler.msg_receive_queues[IDs.GET_PAUSE.value]

    odom.start()

    time.sleep(1)
    try:
        while 1:
            if len(pause_queue) > 0:
                data = pause_queue.pop()
                if data[0]:
                    paused.set()
                else:
                    paused.clear()
            # if len(can_handler.msg_receive_queues[IDs.GET_ODOM.value]) > 0:
            #     odom_msg = can_handler.msg_receive_queues[IDs.GET_ODOM.value].pop()
            #     #print(odom_msg)
                
            #     [x, y, theta, left, right, trans, ang, gyr_ang] = struct.unpack('8f', odom_msg.data)
        

            #     print(f"ID:{hex(odom_msg.arbitration_id)}, x:{x:.2f}, y:{y:.2f}, theta:{theta*180/math.pi:.2f}, l_speed:{left:.2f}, r_speed:{right:.2f}, trans:{trans:.2f}, ang:{ang:.2f}")

            time.sleep(0.01)
            # if time.time() - start_time > 5 and not sent:
            #     sent = True
            #     can_handler.msg_send_queues[IDs.GET_POSITION.name].put((IDs.GET_POSITION, []))
            
    except KeyboardInterrupt:
        pass

    can_handler.stop_threads()


if __name__ == "__main__":
    main_func()
    