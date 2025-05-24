
class FrontCenterGripper:
    OPEN = 157
    NEUTRAL = 150
    GRIP = 140
    CLOSED = 120


class VacuumLift:
    UP = 290
    HOLD = 230
    PUSH = 147
    HOVER = 120
    DROP1 = 105      # Position for dropping one plank
    PICKUP2 = 95  # Position for pickinng-up one plank
    POSITION2 = 40
    DROP2 = 10
    DOWN = 0


class CenterLift:
    UP = 600  # 265*3
    LIFT2 = 550  # 200*3
    DROP1 = 160*3
    POSITION2 = 360  # 140*3
    DROP2 = 140*3
    HOLD2 = 240  # 90*3
    HOVER = 15*3
    DOWN = 9


class CenterSwing:
    UP = 240
    INIT = 195
    DOWN = 150


class Vacuum:
    UP = 240
    PUSH = 195
    DROP = 175
    MIDDLE = 150
    DOWN = 60


class FrontGripLift:
    UP = 300
    HOLD = 210
    HOVER = 20
    DOWN = 0


class BackSwing:
    HOLD = 130
    BANNER = 140
    PICK = 150
    DROP = 165
