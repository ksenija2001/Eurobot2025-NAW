import pigpio
from enum import Enum
import can
import struct
import math
import time
from robot_pkg.can_controller import CanNetwork, IDs
from robot_pkg.logger import LogHandler
from robot_pkg.odometry import OdometryHandler, Odometry
import logging
from collections import deque

LOW = 300 #us  = 0.3 ms
HIGH = 2000 #us  = 2 ms

class Channels(Enum):
    STEER = 1
    THROTTLE = 3
    REVERSE = 5    # switch
    SPEED = 6      # switch

MAX_RPM = 12100

class PPM_Receiver:

    def __init__(self, in_gpio, num_of_channels, can_queue:deque, logger:logging.Logger):
        self.log = logger

        self.in_gpio = in_gpio
        self.num_of_channels = num_of_channels
        self.queue = can_queue

        self.last_tick = None
        self.start_of_frame = False
        self.channel = 0
        self.pi_gpio = None
        self.callback = None
        self.channels = {i:1000 for i in range(1,self.num_of_channels+1)} 

        self.rpm_left = 0
        self.rpm_right = 0

    def start(self):
        try:
            self.pi_gpio = pigpio.pi()
            if not self.pi_gpio.connected:
                raise Exception

            self.pi_gpio.set_mode(self.in_gpio, pigpio.INPUT)
        except Exception as e:
            print(e)
            exit()

        self.callback = self.pi_gpio.callback(self.in_gpio, pigpio.FALLING_EDGE, self.ppm_callback)
        self.log.info(f"Started NFS callback")

    def stop(self):
        self.callback.cancel()
        self.pi_gpio.stop()

        self.log.info(f"Stopped NFS callback")

    # Called on every falling edge of self.in_gpio
    def ppm_callback(self, gpio, level, tick):
        if self.last_tick is not None:
            diff = pigpio.tickDiff(self.last_tick, tick)
            if diff > HIGH: # start of frame
                self.ppm_to_rpm()
                self.start_of_frame = True
                self.channel = 1
            else:
                if self.start_of_frame:
                    if self.channel <= self.num_of_channels:
                        self.channels[self.channel] = diff
                        self.channel += 1
        self.last_tick = tick

    def separate_rpm(self, actual_rpm, throttle, steer, speed, direction):
        rpm = throttle/100 * speed * (-1)**(not direction) + steer/100 * speed/5
        sign = -1 if rpm < 0 else 1
        
        if abs(rpm) > MAX_RPM:
            rpm = sign * MAX_RPM

        ramp = speed * 0.01
        if rpm != 0 and actual_rpm + ramp < rpm:
            actual_rpm += ramp
        elif rpm != 0 and actual_rpm - ramp > rpm:
            actual_rpm -= ramp
        else:
            actual_rpm = rpm

        return (int)(actual_rpm)

    def ppm_to_rpm(self):
        direction = self.channels[Channels.REVERSE.value] < 1500  # 1 - forward, 0 - reverse

        if self.channels[Channels.SPEED.value] < 1250:
           speed_factor =  0.1
        elif self.channels[Channels.SPEED.value] < 1750:
            speed_factor =  0.5
        else:
            speed_factor =  0.9

        throttle = 0.1 * (self.channels[Channels.THROTTLE.value] - 1000)
        speed = MAX_RPM * speed_factor * (throttle > 0.0)

        steer = (int)(0.2 * (self.channels[Channels.STEER.value] - 1000) - 100)

        self.rpm_left = self.separate_rpm(self.rpm_left, 
                                     throttle,
                                     steer,
                                     speed,
                                     direction)
        self.rpm_left = self.separate_rpm(self.rpm_right, 
                                     throttle,
                                     -steer,
                                     speed,
                                     direction)

        self.log.debug(f"{Channels.REVERSE.value}:{(int)(direction)} {Channels.SPEED.value}:{(int)(speed):5d} {Channels.THROTTLE.value}:{(int)(throttle):3d} {Channels.STEER.value}:{(int)(steer):4d}")
        self.log.info(f"RPM_left:{self.rpm_left:6d} RPM_right:{self.rpm_right:6d}\n")

        nfs_msg = struct.pack('4i', self.rpm_left, self.rpm_right, direction, direction)
        self.queue.append(nfs_msg)


if __name__ == "__main__":
    log_handler = LogHandler()
    nfs_log = log_handler.get_loggers("nfs")
    can_log = log_handler.get_logger("can")
    odom_log = log_handler.get_logger("odom")

    can_handler = CanNetwork(channel='can0', interface='socketcan', max_queue_size=10, log=can_log)
    can_handler.start_threads()

    odom = OdometryHandler(can_handler.msg_receive_queues[IDs.GET_ODOM.value],
                           can_handler.msg_send_queues[IDs.RESET_ODOM.value],
                           odom_log,
                           Odometry(0.0, 0.0, 90*math.pi/180))

    odom.start()

    time.sleep(1)

    rec = PPM_Receiver(17, 8, nfs_log, can_handler.msg_send_queues[IDs.SET_MOTOR_RPM.value])
    rec.start()

    try:
        while 1:
            time.sleep(0.001)
            
    except KeyboardInterrupt:
        rec.stop()