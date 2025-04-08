from collections.abc import Iterable
from typing import Any
from enum import Enum
from robot_pkg.step import Step
from robot_pkg.conditions import ConditionType, Condition

class Color(Enum):
    BLUE   = 'blue'
    YELLOW = 'yellow'

    def __eq__(self, other:str):
        return self.name.lower() == other

class Square(Enum):
    UPPER  = 'upper'
    CENTER = 'center'
    LOWER  = 'lower'

    def __eq__(self, other:str):
        return self.name.lower() == other


class Mood(Enum):
    PASSIVE   = 'passive'
    AGGRESSIVE = 'aggressive'

    def __eq__(self, other:str):
        return self.name.lower() == other

class Strategy:
    def __init__(self, color:str, square:str, mood:str):
        self.color  = Color(color).name
        self.square = Square(square).name
        self.mood   = Mood(mood).name

        self.steps:list[Step] = []

    def __eq__(self, other):
        return self.color == other.color and self.square == other.square and self.mood == other.mood

    def __call__(self, sensors=None, task_steps:list=None, ID=None, m=None, a:list=[], s:list=[], c:list[Condition]=[], p=0) -> Any:
        if task_steps is None:
            if ConditionType.CINCH not in [cond._type for cond in c]:
                if m != None and ConditionType.POSITION not in [cond._type for cond in c]:
                    c.append(Condition.InPosition(None))
                
                if len(s) > 0:
                    c.append(Condition.ServoMoving(None))

                if ID != 100:
                    c.append(Condition.MatchTime(100, 96)) 

            servos = []
            for servo in s:
                if type(servo) is tuple:
                    servos.extend(servo)
                else:
                    servos.append(servo)

            step = Step(ID, m, a, servos, c, p)
            self.steps.append(step)

            c.clear() # conditions are cleared before next step
            s.clear()
            a.clear()
        else:
            task_steps[0].ID = ID
            self.steps.extend(task_steps)
    
    def __repr__(self):
        return f"Color: {self.color}\nSquare: {self.square}\nMood: {self.mood}\n---------------------------------------"




    
