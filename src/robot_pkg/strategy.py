from collections.abc import Iterable
from typing import Any
from robot_pkg.data import Color, Square, Mood
from robot_pkg.step import Step
from robot_pkg.conditions import ConditionType, Condition

class Strategy:
    def __init__(self, color:str, square:str, mood:str):
        self.color  = Color(color).name
        self.square = Square(square).name
        self.mood   = Mood(mood).name

        self.steps:list[Step] = []

    def __eq__(self, other):
        return self.color == other.color and self.square == other.square and self.mood == other.mood

    def __call__(self, ID=None, m=None, a:list=[], s:list=[], c:list[Condition]=[], p=0) -> Any:
        if ConditionType.CINCH not in [cond_type for cond_type, id in c]:
            if m != None:
                c.append((ConditionType.POSITION,))
            
            if len(s) > 0:
                c.append((ConditionType.SERVO,))

        step = Step(ID, m, a, s, c, p)
        self.steps.append(step)

        c.clear() # conditions are cleared before next step
    
    def __repr__(self):
        return f"Color: {self.color}\nSquare: {self.square}\nMood: {self.mood}\n---------------------------------------"

    # TODO taskovi u folderu kao male strategije, extend tih stepova ovde



    
