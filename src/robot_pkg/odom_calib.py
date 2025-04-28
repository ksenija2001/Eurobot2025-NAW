from robot_pkg.move import Move
from robot_pkg.consts import ODOM_DATA, IDs
from robot_pkg.main import log_handler, can_handler


import time, struct, math

def odom_calib():
    main_log = log_handler.get_logger("main")

    # Open can socket and start sending and receiving threads
    can_handler.start_threads()
    Move.start_threads()

    last_left_gain = 0
    last_right_gain = 0
    # wheel->inc_mm = (wheel->diameter*M_PI)/PPR;

    left_gain = ODOM_DATA.left_gain #ODOM_DATA.odom_left_diameter * math.pi / 8192
    right_gain = ODOM_DATA.right_gain #ODOM_DATA.odom_right_diameter * math.pi / 8192

    # ODOM_DATA.inc_mm = ODOM_DATA.diameter * math.pi / 8192

    packed = [ODOM_DATA.left_gain, ODOM_DATA.right_gain, ODOM_DATA.inc_mm, ODOM_DATA.odom_track]
    data = struct.pack('4f', *packed)
    can_handler.msg_send_queues[IDs.ODOM_CONFIG.value].append(data)

    # while abs(left_gain - last_left_gain) > 1e-4 or abs(right_gain - last_right_gain) > 1e-4:
    #     reset = Move.ResetOdom(500, 0, 1.5707)
    #     reset._execute()

    #     Move.move_done.wait()

    #     move = Move.To(500, 1000, 'f', 150, 250, 0.5, 0.5)
    #     move._execute()

    #     Move.move_done.wait()

    #     # time.sleep(10)

    #     move = Move.Rotate(3.1415, 0.5, 0.5)
    #     move._execute()

    #     Move.move_done.wait()

    #     move = Move.To(500, 200, 'f', 150, 250, 0.5, 0.5)
    #     move._execute()

    #     Move.move_done.wait()

    #     move = Move.Rotate(-3.1415, 0.5, 0.5)
    #     move._execute()

    #     Move.move_done.wait()

    #     move = Move.Distance(-250, 150, 250)
    #     move._execute()

    #     Move.move_done.wait()

    #     time.sleep(1)

    #     left = Move.pose.left_inc
    #     right = Move.pose.right_inc

    #     main_log.info(f"\n-------------------\nOLD GAIN: {left_gain}, {right_gain}")

    #     total_distance = (left * left_gain + right * right_gain)
    #     delta_angle = (left * left_gain - right * right_gain)
    #     factor = delta_angle / total_distance

    #     last_left_gain = left_gain
    #     last_right_gain = right_gain

    #     left_gain = (1. - factor) * left_gain
    #     right_gain = (1. + factor) * right_gain

    #     main_log.info(f"NEW GAIN: {left_gain}, {right_gain}")

    #     packed = [left_gain, right_gain, ODOM_DATA.inc_mm, ODOM_DATA.odom_track]
    #     data = struct.pack('4f', *packed)
    #     can_handler.msg_send_queues[IDs.ODOM_CONFIG.value].append(data)

    #     input("Continue...")


    last_inc_mm = 0
    inc_mm = ODOM_DATA.inc_mm

    time.sleep(3)
    while abs(inc_mm - last_inc_mm) > 1e-4:
        reset = Move.ResetOdom(500, 0, 1.5707)
        reset._execute()

        Move.move_done.wait()

        move = Move.To(500, 2000, 'f', 150, 250, 0.5, 0.5)
        move._execute()

        Move.move_done.wait()

        time.sleep(1)

        real_distance = float(input("Real Distance: "))

        last_inc_mm = inc_mm

        main_log.info(f"\n-------------------\nOLD INC_MM: {inc_mm}")

        inc_mm = last_inc_mm * (real_distance / 2000)

        main_log.info(f"\n-------------------\nNEW INC_MM: {inc_mm}")

        packed = [ODOM_DATA.left_gain, ODOM_DATA.right_gain, inc_mm, ODOM_DATA.odom_track]
        data = struct.pack('4f', *packed)
        can_handler.msg_send_queues[IDs.ODOM_CONFIG.value].append(data)

        input("Continue: ")

    main_log.info(f"\n-------------------\nFINAL INC_MM: {ODOM_DATA.inc_mm}")

    ODOM_DATA.inc_mm = inc_mm

    track = ODOM_DATA.odom_track
    last_track = 0

    time.sleep(3)
    while abs(track - last_track) > 1e-4:
        count = int(input("Number of turns: "))
        if count == 0:
            break
        reset = Move.ResetOdom(500, 0, 1.5707)
        reset._execute()

        Move.move_done.wait()

        move = Move.Distance(250, 150, 250)
        move._execute()
        Move.move_done.wait()

        start_angle = Move.pose.theta

        move = Move.Rotate(-math.pi*2*count, 0.5, 0.5)
        move._execute()

        Move.move_done.wait()

        move = Move.Distance(-300, 150, 250)
        move._execute()

        Move.move_done.wait()

        time.sleep(1)

        delta_angle = (Move.pose.theta - start_angle)

        main_log.info(f"\n-------------------\nOLD TRACK: {track}")

        last_track = track
        track = last_track * (1 - (delta_angle / (2. * math.pi * count)))

        main_log.info(f"\n-------------------\nNEW TRACK: {track}")

        packed = [ODOM_DATA.left_gain, ODOM_DATA.right_gain, ODOM_DATA.inc_mm, track]
        data = struct.pack('4f', *packed)
        can_handler.msg_send_queues[IDs.ODOM_CONFIG.value].append(data)
    
    ODOM_DATA.odom_track = track

    # main_log.info(f"FINAL GAIN: {left_gain}, {right_gain}")

    # ODOM_DATA.left_gain = left_gain
    # ODOM_DATA.right_gain = right_gain


    stop = Move.Stop()
    stop._execute()
    time.sleep(1)

    Move.stop_threads()
    can_handler.stop_threads()



if __name__ == "__main__":
    odom_calib()




