from robot_pkg.move import Position

class Area:
    YELLOW_HOME = Position(150 +225, 2000-225,  1.57)
    YELLOW_4    = Position(550 +225,       75, -1.57)
    YELLOW_2    = Position(1000+225,      225, -1.57)
    YELLOW_5    = Position(3000-225,       75, -1.57)
    YELLOW_3    = Position(3000-225, 1100-225,     0)

    BLUE_HOME  = Position(2850-225, 2000-225,  1.57)
    BLUE_5     = Position(     225,       75, -1.57)
    BLUE_3     = Position(     225, 1100-225,  3.14)
    BLUE_4     = Position(2000+225,       75, -1.57)
    BLUE_2     = Position(2000-225,      225, -1.57)


class MaterialStack:
    STACK1  = Position(2175, 1725,  1.57)
    STACK2  = Position( 825, 1725,  1.57)

    STACK3  = Position(  75, 1325,  3.14)
    STACK4  = Position(  75,  400,  3.14)
    STACK5  = Position( 775,  250, -1.57)
    STACK9  = Position(1100,  950,  1.57)

    STACK8  = Position( 3000-75, 1325,     0)
    STACK7  = Position( 3000-75,  400,     0)
    STACK6  = Position(3000-775,  250, -1.57)
    STACK10 = Position(    1900,  950,  1.57)

if __name__ == "__main__":

    print(Area.YELLOW_HOME)
    print(MaterialStack.STACK1)