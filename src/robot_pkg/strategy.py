from collections.abc import Iterable
from typing import Any
from enum import Enum
from robot_pkg.step import Step
from robot_pkg.move import Move, MoveType
from robot_pkg.conditions import ConditionType, Condition



class Color(Enum):
    BLUE = 'blue'
    YELLOW = 'yellow'

    def __eq__(self, other: str):
        return self.name.lower() == other


class Square(Enum):
    UPPER = 'upper'
    CENTER = 'center'
    LOWER = 'lower'

    def __eq__(self, other: str):
        return self.name.lower() == other


class Mood(Enum):
    PASSIVE = 'passive'
    AGGRESSIVE = 'aggressive'
    SEMI = 'semi'
    RUDE = 'rude'

    def __eq__(self, other: str):
        return self.name.lower() == other


class Strategy:

    def __init__(self, color: str = "yellow", square: str = "upper", mood: str = "passive"):
        self.color = Color(color).name
        self.square = Square(square).name
        self.mood = Mood(mood).name

        self.reached_home_step = False

        self.steps: list[Step] = []

    def __eq__(self, other):
        return self.color == other.color and self.square == other.square and self.mood == other.mood

    def __call__(self, task_steps: list = None, ID=None, m: Move = None, a: list = [], s: list = [], c: list[Condition] = [], sima_id: int = None, sima: list = [], p=0) -> Any:
        if task_steps is None:
            if ConditionType.CINCH not in [cond._type for cond in c]:
                if len(s) > 0:
                    c.append(Condition.ServoMoving(None))

                # Don't add to empty steps and to steps after home
                if (m is not None or len(s) > 0):
                    c.append(Condition.MatchTime(100, 96))

                if m is not None and ConditionType.POSITION not in [cond._type for cond in c]:
                    c.append(Condition.InPosition(None))

            servos = []
            for servo in s:
                if type(servo) is tuple:
                    servos.extend(servo)
                else:
                    servos.append(servo)

            step = Step(ID, m, a, servos, c, sima_id, sima, p)
            self.steps.append(step)

            c.clear()  # conditions are cleared before next step
            # s.clear()
            # a.clear()
        else:
            if m is not None:
                servos = []
                for servo in s:
                    if type(servo) is tuple:
                        servos.extend(servo)
                    else:
                        servos.append(servo)

                step = Step(ID, m, a, servos, c, sima_id, sima, p)
                self.steps.append(step)

                position_cond = [
                    cond for cond in c if cond._type is ConditionType.POSITION]

                if len(position_cond) > 0:
                    task_steps[-1].conditions.append(position_cond[0])
                else:
                    task_steps[-1].conditions.append(
                        Condition.InPosition(None))
            else:
                task_steps[0].ID = ID

            self.steps.extend(task_steps)

    def __repr__(self):
        return f"Color: {self.color}\nSquare: {self.square}\nMood: {self.mood}\n---------------------------------------"
