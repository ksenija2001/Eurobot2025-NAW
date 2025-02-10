from threading import Thread
import struct
import math
import time

class Odometry:
    x:float
    y:float
    theta:float
    left_speed:float
    right_speed: float
    linear_speed: float
    angular_speed:float
    gyr_angular_speed:float

    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta

class OdometryHandler:

    def __init__(self, rec_queue, send_queue, odom_log, initial_odom:Odometry):
        self.log = odom_log
        self.queue = rec_queue
        self.send_queue = send_queue
        self.odom:Odometry = initial_odom
        self.reset_odom(initial_odom)

        self.running = False
        self._odom_thread = Thread(target=self.receive)

    def start(self):
        self.running = True
        self._odom_thread.start()

        self.log.info(f"Started OdometryHandler")

    def stop(self):
        self.running = False
        self._odom_thread.join()

        self.log.info(f"Stopped OdometryHandler")

    def receive(self):
        while self.running:
            if len(self.queue) > 0:
                odom_msg = self.queue.pop()

                [x, y, theta, left, right, trans, ang, gyr_ang] = struct.unpack('8f', odom_msg.data)

                self.odom.x = x
                self.odom.y = y
                self.odom.theta = theta
                self.odom.left_speed = left
                self.odom.right_speed = right
                self.odom.linear_speed = trans
                self.odom.angular_speed = ang
                self.odom.gyr_angular_speed = gyr_ang

                self.log.debug(f"x:{x:4.2f}, y:{y:4.2f}, theta:{theta*180/math.pi:4.2f}, l_speed:{left:4.2f}, r_speed:{right:4.2f}, trans:{trans:4.2f}, ang:{ang:4.2f}")
            
            time.sleep(0.001)  # 1ms

    def reset_odom(self, odom:Odometry):
        reset_msg = struct.pack('3f', odom.x, odom.y, odom.theta)
        self.send_queue.append(reset_msg)

