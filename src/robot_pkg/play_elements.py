from robot_pkg.move import Position, PositionType

class Area:
    YELLOW_HOME = Position(150 +225, 2000-225,  1.57, PositionType.HOME)
    YELLOW_4    = Position(550 +225,       75, -1.57, PositionType.FIELD)
    YELLOW_2    = Position(1000+225,      225, -1.57, PositionType.FIELD)
    YELLOW_5    = Position(3000-225,       75, -1.57, PositionType.FIELD)
    YELLOW_3    = Position(3000-225, 1100-225,     0, PositionType.FIELD)

    BLUE_HOME  = Position(2850-225, 2000-225,  1.57, PositionType.FIELD)
    BLUE_5     = Position(     225,       75, -1.57, PositionType.HOME)
    BLUE_3     = Position(     225, 1100-225,  3.14, PositionType.FIELD)
    BLUE_4     = Position(2000+225,       75, -1.57, PositionType.FIELD)
    BLUE_2     = Position(2000-225,      225, -1.57, PositionType.FIELD)

    @classmethod
    def get_all_areas(cls):
        '''
        Returns all defined areas on the table.
        
        '''
        return [value for name, value in cls.__dict__.items() if isinstance(value, Position)]
    
    @classmethod
    def get_all_visited_areas(cls, color=None):
        '''
        Gets all VISITED areas on the field. If a colors is passed, returns visited fields of that color. If nothing is passed, returns all fields.
        Can be used for avoiding parts of the table where there is a possibility planks and cans might have fallen while building (if against 
        lower-ranked teams), or to steal (if against better teams).
        
        '''
        areas = [value for name, value in cls.__dict__.items() 
             if isinstance(value, Position) and value.visited]
    
        if color is None:
            return areas
        
        color = color.upper()
        if color not in ('BLUE', 'YELLOW'):
            raise ValueError("Color must be 'BLUE', 'YELLOW', or None")
            
        return [area for name, area in cls.__dict__.items() 
                if isinstance(area, Position) and color in name and area.visited]

class MaterialStack:
    STACK1  = Position(2175, 1725,  1.57, PositionType.STACK)
    STACK2  = Position( 825, 1725,  1.57, PositionType.STACK)

    STACK3  = Position(  75, 1325,  3.14, PositionType.STACK)
    STACK4  = Position(  75,  400,  3.14, PositionType.STACK)
    STACK5  = Position( 775,  250, -1.57, PositionType.STACK)
    STACK9  = Position(1100,  950,  1.57, PositionType.STACK)

    STACK8  = Position( 3000-75, 1325,     0, PositionType.STACK)
    STACK7  = Position( 3000-75,  400,     0, PositionType.STACK)
    STACK6  = Position(3000-775,  250, -1.57, PositionType.STACK)
    STACK10 = Position(    1900,  950,  1.57, PositionType.STACK)
    @classmethod
    def get_all_unvisited_stacks(cls):
        '''
        Gets all stacks that are available.
        
        '''
        return [value for name, value in cls.__dict__.items() if isinstance(value, Position) and not value.visited]
    
    @classmethod
    def get_closest_stack(cls, curr_x, curr_y):
        '''
        Determines the closest available stack relative to the current robot pose.
        '''
        available_stacks = cls.get_all_unvisited_stacks()
        if not available_stacks:
            return None
        return min(available_stacks, key=lambda s: s.distance_to(curr_x, curr_y))

if __name__ == "__main__":
    # Test basic functionality
    print("Yellow home position:", Area.YELLOW_HOME)
    print("Stack1 position:", MaterialStack.STACK1)
    
    # Test closest stack calculation
    closest = MaterialStack.get_closest_stack(1000, 1000)
    print(f"Closest stack to (1000,1000): {closest}")
    
    # Mark a stack as visited and test unvisited
    MaterialStack.STACK1.visited = True
    print("Unvisited stacks:", len(MaterialStack.get_all_unvisited_stacks()))