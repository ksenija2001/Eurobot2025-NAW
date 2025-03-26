from actuators import (
    ActuatorType,
    Actuators
)
from servo import (
    ServoType,
    ServoMoving
)
from conditions import Condition

class SetPosition:
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta
        self.type = MoveType.SET_POSITION
    
    def _execute(self, nucleo:Nucleo):
        nucleo.set_robot_position(self.x, self.y, self.theta)

class Move:
    def __init__(self):
        self._distance = 0
        self._abs_angle = 0
        self._rel_angle = 0
        self._x = 0
        self._y = 0
        self.type = None
    
    @classmethod
    def Distance(cls, distance:float):
        move = cls()
        move.type = MoveType.DISTANCE
        move._distance = distance
        return move

    @classmethod
    def RotateTo(cls, abs_angle:float):
        rotate = cls()
        rotate.type = MoveType.ROTATE_TO
        rotate._abs_angle = abs_angle
        return rotate 

    @classmethod
    def RotateFor(cls, rel_angle:float):
        rotate = cls()
        rotate.type = MoveType.ROTATE_FOR
        rotate._rel_angle = rel_angle
        return rotate

    @classmethod
    def To(cls, x:float, y:float):
        move = cls()
        move.type = MoveType.TO_XY
        move._x = x
        move._y = y
        return move

    def _execute(self, nucleo:Nucleo):
        if self.type == MoveType.DISTANCE:
            nucleo.move_distance(self._distance)
        elif self.type == MoveType.TO_XY:
            nucleo.move_to(self._x, self._y)
        elif self.type == MoveType.ROTATE_TO:
            nucleo.move_rotate_to(self._abs_angle)
        elif self.type == MoveType.ROTATE_FOR:
            nucleo.move_rotate_for(self._rel_angle)

class Vacuum:
    def __init__(self, status:bool):
        self.status = status
        self.type = ActuatorType.VACUUM
    
    def execute(self, actuators:Actuators):
        actuators.vacuum(self.status)

class Servo:

    def __init__(self):
        self._angle = 0
        self._speed = 100
        self._type = None

    @classmethod
    def Fork(cls, angle, speed=50):
        servo = cls()
        servo._type = ServoType.FORK
        servo._angle = angle
        servo._speed = speed
        return servo

    @classmethod
    def ForkLift(cls, angle, speed=100):
        servo = cls()
        servo._type = ServoType.FORK_LIFT
        servo._angle = angle
        servo._speed = speed
        return servo

    @classmethod
    def Arm(cls, angle, speed=100):
        servo = cls()
        servo._type = ServoType.ARM
        servo._angle = angle
        servo._speed = speed
        return servo

    @classmethod
    def Vacuum(cls, angle, speed=100):
        servo = cls()
        servo._type = ServoType.VACUUM
        servo._angle = angle
        servo._speed = speed
        return servo

    @classmethod
    def BucketLift(cls, angle, speed=100):
        servo = cls()
        servo._type = ServoType.BUCKET_LIFT
        servo._angle = angle
        servo._speed = speed
        return servo

    @classmethod
    def BucketHolder(cls, angle, speed=100):
        servo = cls()
        servo._type = ServoType.BUCKET_HOLDER
        servo._angle = angle
        servo._speed = speed
        return servo

    @classmethod
    def BucketSeparator(cls, angle, speed=100):
        servo = cls()
        servo._type = ServoType.BUCKET_SEPARATOR
        servo._angle = angle
        servo._speed = speed
        return servo

    def _execute(self, servo_moving:ServoMoving):
        servo_moving.set_angles(self._type.value, self._angle, self._speed)

    def __repr__(self):
        return str(self._type.value) + " " + str(self._angle)


class Step:
    def __init__(self, ID, movement:Move, actuation:Vacuum, servos:list[Servo], conditions:list[tuple], points):
        self.ID = ID
        self.movement = movement
        self.actuation = actuation
        self.servos = servos
        self.conditions = []
        self.points = points
        for cond in conditions:
            self.conditions.append(Condition(cond))

    # def step(self, nucleo:Nucleo, actuators:Actuators, servo_moving:ServoMoving):
    #     if self.movement is not None:
    #         print(f"Executing movement {self.movement.type}")
    #         self.movement._execute(nucleo)

    #     if self.actuation is not None:
    #         print(f"Executing actuator {self.actuation.type}")
    #         self.actuation.execute(actuators)

    #     for servo in self.servos:
    #         print(f"Executing servo {servo._type}")
    #         servo._execute(servo_moving)

        
