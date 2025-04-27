from robot_pkg.move import Move
from robot_pkg.consts import ODOM_DATA, IDs
from robot_pkg.logger import LogHandler
log_handler = LogHandler()

from robot_pkg.can_controller import CanNetwork
can_handler = CanNetwork(channel='can0', interface='socketcan', max_queue_size=10)
can_handler.init_queues(10)

import time, struct, math

main_log = log_handler.get_logger("main")

# Open can socket and start sending and receiving threads
can_handler.start_threads()
Move.start_threads()

last_left_gain = 0
last_right_gain = 0
# wheel->inc_mm = (wheel->diameter*M_PI)/PPR;

left_gain = ODOM_DATA.odom_left_diameter * math.pi / 8192
right_gain = ODOM_DATA.odom_right_diameter * math.pi / 8192

while abs(left_gain - last_left_gain) > 1e-6 or abs(right_gain - last_right_gain) > 1e-6:
    reset = Move.ResetOdom(0, 0, 1.5707)
    reset._execute()

    Move.move_done.wait()

    move = Move.To(0, 1500, 'f', 300, 500, 1, 1)
    move._execute()

    Move.move_done.wait()

    move = Move.Rotate(3.1415, 1, 1)
    move._execute()

    Move.move_done.wait()

    move = Move.To(0, 200, 'f', 300, 500, 1, 1)
    move._execute()

    Move.move_done.wait()

    move = Move.Rotate(-3.1415, 1, 1)
    move._execute()

    Move.move_done.wait()

    move = Move.Distance(-250, 300, 500)
    move._execute()

    Move.move_done.wait()

    time.sleep(1)

    left = Move.pose.left_inc
    right = Move.pose.right_inc

    # total_distance is the average between the left wheel and right wheel
    # distance (0.5 * (left_pulse * left_gain + right_pulse * right_gain)).
    # delta_angle is the difference between the pulse count of the left wheel
    # and the right wheel, also taking wheel gain into account.

    main_log.info(f"-------------------\nOLD GAIN: {left_gain}, {right_gain}")
    
    total_distance = (left * left_gain + right * right_gain)/2.0
    delta_angle = (left * left_gain - right * right_gain) #/2.0
    factor = delta_angle / total_distance

    last_left_gain = left_gain
    last_right_gain = right_gain

    left_gain = (1. + factor) * left_gain;
    right_gain = (1. - factor) * right_gain;

    main_log.info(f"NEW GAIN: {left_gain}, {right_gain}\n-------------------")

    # Update odometry wheel diameters
    left_diam = left_gain * 8192 / math.pi
    right_diam = right_gain * 8192 / math.pi
    data = struct.pack('3f', left_diam, right_diam, ODOM_DATA.odom_track)
    can_handler.msg_send_queues[IDs.ODOM_CONFIG.value].append(data)

    input("Continue...")

stop = Move.Stop()
stop._execute()
time.sleep(1)

Move.stop_threads()
can_handler.stop_threads()




