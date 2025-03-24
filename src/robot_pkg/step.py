import struct, time
from threading import Thread, Event
from collections import deque

from robot_pkg.main import can_handler, paused
from robot_pkg.consts import IDs

class Servo:
    @classmethod
    def RightVacuumLift(cls, position:int, speed:int):
        


class Move:
    ack_queue = can_handler.msg_receive_queues[IDs.GET_MOVE_DONE.value]
    send_queue:deque
    data:bytes

    def wait(self, done:Event, success:Event, timeout, dt, attemp=3):
        start_time = time.time()
        while time.time() - start_time > timeout:
            # Waiting for movement to finish
            if len(self.ack_queue) > 0:
                self.ack_queue.pop()
                success.set()
                break
            
            # Check if pause was activated
            if paused.is_set():
                if attemp > 0:   # Try again given number of times to finish movement
                    attemp -= 1
                    move_back, move_success = Move.Distance(-100, attempt=0)
                
                    if move_back.wait() and move_success.is_set():
                        # Attempt to move again if backing was successful
                        self.send_queue.append(self.data)
                        start_time = time.time()
                        continue
                
                # If all attempts run out or backing was unsuccessful stop the movement
                break

            time.sleep(dt) 

        done.set()

    def start_wait(self, attempt=3, timeout=5000, dt=0.0001) -> tuple[Event]:
        done = Event()
        success = Event()
        wait_thread = Thread(target=self.wait, args=(done, success, timeout, dt, attempt))
        wait_thread.start()

        return done, success

    @classmethod
    def RPM(cls, left_rpm:int, right_rpm:int) -> Event:
        '''
            Sets target speed[RPM] for both motors.
        '''
        cls.data = struct.pack('2i', left_rpm, right_rpm)
        cls.send_queue = can_handler.msg_send_queues[IDs.SET_MOTOR_RPM.value]
        cls.send_queue.append(cls.data)

        return cls.start_wait()

    @classmethod
    def Speed(cls, left_speed:int, right_speed:int) -> Event:
        '''
            Sets target speed[mm/s] for both motors.
        '''
        cls.data = struct.pack('2i', left_speed, right_speed)
        cls.send_queue = can_handler.msg_send_queues[IDs.SET_MOTOR_SPEED.value]
        cls.send_queue.append(cls.data)

        return cls.start_wait()

    @classmethod
    def Distance(cls, p:float, v:float, a:float, attempt=3, max_vel:float=1200, max_acc:float=2000) -> tuple[Event, Event]:
        '''
            Starts relative movement of distance[mm] from current robot position 
            with respect to velocity and acceleration limits.
        '''
        cls.data = struct.pack('5f', p, v, a, max_vel, max_acc)
        cls.send_queue = can_handler.msg_send_queues[IDs.SET_DISTANCE.value]
        cls.send_queue.append(cls.data)

        return cls.start_wait(attempt=attempt)

    @classmethod
    def To(cls, x_coor:float, y_coor:float, direction:bool, max_vel:float=1200, max_acc:float=2000, max_ang_vel:float=11, max_ang_acc:float=10) -> Event:
        '''
            Starts absolute movement to (x,y) coordinate of table with respect to 
            velocity and acceleration limits.
        '''
        cls.data = struct.pack('2fi4f', x_coor, y_coor, direction, max_vel, max_acc, max_ang_vel, max_ang_acc)
        cls.send_queue = can_handler.msg_send_queues[IDs.SET_XY.value]
        cls.send_queue.append(cls.data)

        return cls.start_wait()

    @classmethod
    def Rotate(cls, theta:float, max_ang_vel:float=11, max_ang_acc:float=10) -> Event:
        '''
            Starts relative rotation of theta[rad] from current orientation of robot 
            with respect to angular velocity and acceleration limits.
        '''
        cls.data = struct.pack('3f', theta, max_ang_vel, max_ang_acc)
        cls.send_queue = can_handler.msg_send_queues[IDs.SET_ROTATION_FOR.value]
        cls.send_queue.append(cls.data)

        return cls.start_wait()

    @classmethod
    def RotateTo(cls, theta:float, max_ang_vel:float=11, max_ang_acc:float=10) -> Event:
        '''
            Starts absolute rotation to theta[rad] with respect to 
            angular velocity and acceleration limits.
        '''
        cls.data = struct.pack('3f', theta, max_ang_vel, max_ang_acc)
        cls.send_queue = can_handler.msg_send_queues[IDs.SET_ROTATION_TO.value]
        cls.send_queue.append(cls.data)

        return cls.start_wait()


if __name__ == "__main__":
    move, success = Move.Distance(100, attemp=2)
    if move.wait() and not success.is_set():
        # Timeout expired, movement unsuccessful
        
        pass
    else:
        # Movement successful
        pass







    
