from enum import EnumDict

class Position:
    x: float                           # x coordinate 
    y: float                           # y coordinate
    h: float                           # default heading

    def __init__(self, x, y, heading):
        self.x = x
        self.y = y
        self.h = heading

    def __call__(self) -> tuple[float, float]:
        return self.x, self.y

class Area(EnumDict):
    YELLOW_CHARGING   = Position(150 +225, 2000-225, -90)
    YELLOW_LEFT_SMALL = Position(550 +225,       75, -90)
    YELLOW_LEFT_BIG   = Position(1000+225,      225, -90)
    YELLOW_RIGHT_SMALL= Position(3000-225,       75, -90)
    YELLOW_RIGHT_BIG  = Position(3000-225, 1100-225,   0)

    BLUE_CHARGING     = Position(2850-225, 2000-225,  -90)
    BLUE_LEFT_SMALL   = Position(     225,       75,  -90)
    BLUE_LEFT_BIG     = Position(     225, 1100-225, -180)
    BLUE_RIGHT_SMALL  = Position(2000+225,       75,  -90)
    BLUE_RIGHT_BIG    = Position(2000-225,      225,  -90)

class MaterialStock(EnumDict):
    YELLOW_RESERVED = Position( 825, 1725,  90)
    BLUE_RESERVED   = Position(2175, 1725,  90)

    LEFT_UPPER      = Position(  75, 1325, 180)
    LEFT_LOWER      = Position(  75,  400, 180)
    LEFT_MIDDLE     = Position( 775,  250, -90)
    LEFT_CENTER     = Position(1100,  950,  90)

    RIGHT_UPPER      = Position(2925, 1325,   0)
    RIGHT_LOWER      = Position(2925,  400,   0)
    RIGHT_MIDDLE     = Position(2225,  250, -90)
    RIGHT_CENTER     = Position(1900,  950,  90)


if __name__ == "__main__":
    c = Position(5, 5, 5)
    print(c())

    print(Area.YELLOW_CHARGING())
    print(MaterialStock.BLUE_RESERVED())