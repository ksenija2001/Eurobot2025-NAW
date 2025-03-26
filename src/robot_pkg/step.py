import struct, time
from threading import Thread, Event
from collections import deque
from enum import Enum

from robot_pkg.main import can_handler, paused
from robot_pkg.consts import IDs

class ServoType(Enum):
    RIGHT_VACUUM_LIFT = 1
    RIGHT_VACUUM = 2
    LEFT_VACUUM_LIFT = 3 
    LEFT_VACUUM = 4    
    RIGHT_GRIP_LIFT = 5  
    LEFT_GRIP_LIFT = 6  
    CENTER_SWING = 7   
    CENTER_LIFT = 8    
    BACK_RIGHT_LIFT = 9  
    BACK_LEFT_LIFT = 10 

def receive(running:Event):
    while running.is_set():
        if len(can_handler.msg_receive_queues[IDs.GET_SERVO_IN_POSITION.value]) > 0:
            servo_msg = can_handler.msg_receive_queues[IDs.GET_SERVO_IN_POSITION.value].pop()

            [id, success] = struct.unpack('2B', servo_msg.data)

            if success:
                Servo.servo_in_position[id] = True
            else:
                # Servo did not reach position
                pass

            print(f"Servo {id}: {success}")
            # self.log.debug(f"Servo {id}: {success}")
        
        if len(can_handler.msg_receive_queues[IDs.GET_SERVO_POSITIONS.value]) > 0:
            servo_msg = can_handler.msg_receive_queues[IDs.GET_SERVO_POSITIONS.value].pop()

            [id, angle_high, angle_low] = struct.unpack('3I', servo_msg.data)

            print(f"{id}: {(int)(angle_high << 8) & angle_low}")
            # self.log.debug(f"{id}: {(int)(angle_high << 8) & angle_low}")

        time.sleep(0.01)  # 10ms



class Servo:
    servo_list:list[int] = []
    servo_in_position = {enum_item.value: True for enum_item in ServoType}
    send_queue:deque = can_handler.msg_send_queues[IDs.SET_SERVO_POSITIONS.value]

    def __init__(self):
        self.id = 0
        self.position = 0
        self.speed = 0

    @classmethod
    def RightVacuumLift(cls, position:int, speed:int):
        servo = cls()
        servo.id = ServoType.RIGHT_VACUUM_LIFT.value
        servo.position = position
        servo.speed = speed

        return servo

    @classmethod
    def RightVacuum(cls, position:int, speed:int):
        servo = cls()
        servo.id = ServoType.RIGHT_VACUUM.value
        servo.position = position
        servo.speed = speed

        return servo

    @classmethod
    def LeftVacuumLift(cls, position:int, speed:int):
        servo = cls()
        servo.id = ServoType.LEFT_VACUUM_LIFT.value
        servo.position = position
        servo.speed = speed

        return servo

    @classmethod
    def LeftVacuum(cls, position:int, speed:int):
        servo = cls()
        servo.id = ServoType.LEFT_VACUUM.value
        servo.position = position
        servo.speed = speed

        return servo

    @classmethod
    def RightGripLift(cls, position:int, speed:int):
        servo = cls()
        servo.id = ServoType.RIGHT_GRIP_LIFT.value
        servo.position = position
        servo.speed = speed

        return servo

    @classmethod
    def LeftGripLift(cls, position:int, speed:int):
        servo = cls()
        servo.id = ServoType.LEFT_GRIP_LIFT.value
        servo.position = position
        servo.speed = speed

        return servo

    @classmethod
    def CenterSwing(cls, position:int, speed:int):
        servo = cls()
        servo.id = ServoType.CENTER_SWING.value
        servo.position = position
        servo.speed = speed

        return servo

    @classmethod
    def CenterLift(cls, position:int, speed:int):
        servo = cls()
        servo.id = ServoType.CENTER_LIFT.value
        servo.position = position
        servo.speed = speed

        return servo

    @classmethod
    def BackRightLift(cls, position:int, speed:int):
        servo = cls()
        servo.id = ServoType.BACK_RIGHT_LIFT.value
        servo.position = position
        servo.speed = speed

        return servo

    @classmethod
    def BackLeftLift(cls, position:int, speed:int):
        servo = cls()
        servo.id = ServoType.BACK_LEFT_LIFT.value
        servo.position = position
        servo.speed = speed

        return servo

    def _execute(self):
        while not Servo.servo_in_position[self.id]:
            pass

        Servo.servo_list.extend([self.id, self.position, self.speed])
        Servo.servo_in_position[self.id] = False

    @classmethod
    def send_positions(cls):
        print(f"list: {Servo.servo_list}")
        size = len(Servo.servo_list)//3
        Servo.servo_list.insert(0, size)
        fmt = ">B" + "BHB"*size 
        servo_msg = struct.pack(fmt, *Servo.servo_list)
        Servo.send_queue.append(servo_msg)
        Servo.servo_list.clear()

    def _check_position(self):
        queue = can_handler.msg_send_queues[IDs.GET_SERVO_POSITIONS.value]
        data = struct.pack('B', self.id)
        queue.append(data)
    

class Move:
    ack_queue = can_handler.msg_receive_queues[IDs.GET_MOVE_DONE.value]
    send_queue:deque
    data:bytes

    @classmethod
    def RPM(cls, left_rpm:int, right_rpm:int):
        '''
            Sets target speed[RPM] for both motors.
        '''
        cls.data = struct.pack('2i', left_rpm, right_rpm)
        cls.send_queue = can_handler.msg_send_queues[IDs.SET_MOTOR_RPM.value]

    @classmethod
    def Speed(cls, left_speed:int, right_speed:int):
        '''
            Sets target speed[mm/s] for both motors.
        '''
        cls.data = struct.pack('2i', left_speed, right_speed)
        cls.send_queue = can_handler.msg_send_queues[IDs.SET_MOTOR_SPEED.value]

    @classmethod
    def Distance(cls, p:float, v:float, a:float):
        '''
            Starts relative movement of distance[mm] from current robot position 
            with respect to velocity and acceleration limits.
        '''
        cls.data = struct.pack('5f', p, v, a)
        cls.send_queue = can_handler.msg_send_queues[IDs.SET_DISTANCE.value]

    @classmethod
    def To(cls, x_coor:float, y_coor:float, direction:bool, v:float, a:float, w:float, alpha:float):
        '''
            Starts absolute movement to (x,y) coordinate of table with respect to 
            velocity and acceleration limits.
        '''
        cls.data = struct.pack('ffiffff', x_coor, y_coor, direction, v, a, w, alpha)
        cls.send_queue = can_handler.msg_send_queues[IDs.SET_XY.value]

    @classmethod
    def Rotate(cls, theta:float, w:float, alpha:float):
        '''
            Starts relative rotation of theta[rad] from current orientation of robot 
            with respect to angular velocity and acceleration limits.
        '''
        cls.data = struct.pack('3f', theta, w, alpha)
        cls.send_queue = can_handler.msg_send_queues[IDs.SET_ROTATION_FOR.value]

    @classmethod
    def RotateTo(cls, theta:float, w:float, alpha:float):
        '''
            Starts absolute rotation to theta[rad] with respect to 
            angular velocity and acceleration limits.
        '''
        cls.data = struct.pack('3f', theta, w, alpha)
        cls.send_queue = can_handler.msg_send_queues[IDs.SET_ROTATION_TO.value]
    
    def _execute(self):
        self.send_queue.append(self.data)


if __name__ == "__main__":
    running = Event()
    running.set()
    thread = Thread(target=receive, args=(running,))
    thread.start()

    can_handler.start_threads()

    servo1 = Servo.RightVacuumLift(100, 10)
    servo2 = Servo.RightVacuum(150, 10)

    servo1._execute()
    servo2._execute()

    Servo.send_positions()

    time.sleep(5)

    servo1 = Servo.RightVacuumLift(150, 10)
    servo2 = Servo.RightVacuum(100, 10)

    servo1._execute()
    servo2._execute()

    Servo.send_positions()

    # servo1._check_position()
    
    time.sleep(5)


    running.clear()
    thread.join()

    can_handler.stop_threads()







    
