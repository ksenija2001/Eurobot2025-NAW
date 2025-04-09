from enum import Enum
from robot_pkg.consts import Variables
import time

class ConditionType(Enum):
    TIMEOUT    = 0      # Wait for defined amount of time, delay
    TIME       = 1      # Check if match time has run out
    POSITION   = 2      # Wait for movement to finish
    END_SWITCH = 3      
    CINCH      = 4      # Wait for cinch to be pulled 
    DETECTION  = 5      # React to a detection
    SERVO      = 6      # Wait for servo to finish moving
    FRONT      = 7      # Check if there are cans in the front
    BACK       = 8      # Check if there are cans in the back

# def timeout(timeout, args) -> bool:
#     start_time = args[0]
#     curr_time  = args[1]
#     if curr_time - start_time >= timeout:
#         return True
#     return False

# def match_time(_time, args) -> bool:
#     if Variables.match_start_time  + _time < args[1]:
#         return True
#     return False

# def position(tmp, args) -> bool:
#     move_done = args[2]
#     if move_done:
#         return True
#     return False

# def cinch(tmp, args) -> bool:
#     curr_cinch = args[3]
#     if not curr_cinch:
#         Variables.match_start_time = time.time()
#         return True
#     return False

# def servo(tmp, args) -> bool:
#     curr_pos = args[4]
#     if curr_pos:
#         return True
#     return False


# conditions = {
#     ConditionType.TIMEOUT : timeout,
#     ConditionType.TIME    : match_time, 
#     ConditionType.POSITION: position,
#     ConditionType.CINCH   : cinch,
#     ConditionType.SERVO   : servo,
# }

class Condition:

    def __init__(self): 
        self._type = None
        self.ID = None
        self.servo_id = 0
        self.servo_position = 0
        self.time = 0

    def check(self, args:list):
        '''
            args: step_start_time, current_time, move_done, cinch_state, servos_moving, front_sensor_state, back_sensor_state
        '''
        if self._type == ConditionType.TIME:
            if args[1] - Variables.match_start_time >= self.time:
                return self.ID
        elif self._type == ConditionType.TIMEOUT:
            if args[1] - args[0] >= self.time:
                return self.ID
        elif self._type == ConditionType.POSITION:
            if args[2]:
                return self.ID
        elif self._type == ConditionType.CINCH:
            if not args[3]:
                Variables.match_start_time = time.time()  # sets the match start time
                return self.ID
        elif self._type == ConditionType.SERVO:
            if args[4]:
                return self.ID
        elif self._type == ConditionType.FRONT:
            if not args[5]:
                return self.ID
        elif self._type == ConditionType.BACK:
            if not args[6]:
                return self.ID
        return False
        # if conditions[self.type](self.value, args):
        #     return self.ID
        # return False
    
    def __repr__(self):
        return str(self._type) + " " + str(self.ID) #+ " " + str(self.value)

    @classmethod
    def MatchTime(cls, step_id:int, time:int):
        '''
            Checks if time[s] has passed since start of match. 
            If it has, jumps to step_id. 
        '''
        condition = cls()
        condition.ID = step_id
        condition.time = time
        condition._type = ConditionType.TIME
        return condition
    
    @classmethod
    def Timeout(cls, step_id:int, time:int):
        '''
            Checks if time[s] has passed since start of step.
            if it has, jumps to step_id.
        '''
        condition = cls()
        condition.ID = step_id
        condition.time = time
        condition._type = ConditionType.TIMEOUT
        return condition
    
    @classmethod
    def CinchPulled(cls, step_id:int):
        '''
            Checks if cinch was pulled.
            If it was, jumps to step_id.
        '''
        condition = cls()
        condition.ID = step_id
        condition._type = ConditionType.CINCH
        return condition

    @classmethod
    def ServoMoving(cls, step_id:int):
        '''
            Checks if all moved servos are in position.
            If they aren't, jumps to step_id.
        '''
        condition = cls()
        condition.ID = step_id
        condition._type = ConditionType.SERVO
        return condition
    
    @classmethod
    def InPosition(cls, step_id:int):
        '''
            Checks if movement has finished.
            If it hasn't, jumps to step_id.
        '''
        condition = cls()
        condition.ID = step_id
        condition._type = ConditionType.POSITION
        return condition

    @classmethod
    def ServoPosition(cls, step_id:int, servo_id:int, servo_position:int):
        '''
            Checks if servo with ID servo_id is in servo_position before start of next step.
            If it isn't, jump to step_id.
        '''
        condition = cls()
        condition.ID = step_id
        condition.servo_id = servo_id
        condition.servo_position = servo_position
        condition._type = ConditionType.SERVO_POSITION
        return condition

    # @classmethod
    # def Detection(cls, step_id:int):
    #     '''
    #         Checks if all moved servos are in position.
    #         If they aren't, jumps to step_id.
    #     '''
    #     condition = cls()
    #     condition.ID = step_id
    #     condition._type = ConditionType.DETECTION
    #     return condition


if __name__ == "__main__":
    cond = Condition((ConditionType.POSITION,))
    print(cond)