
class ElementPosition:
    def __init__(self, x: float = 0, y: float = 0, theta: float = 0):
        self.x = x
        self.y = y
        self.theta = theta

        self.visited = False

    def distance_to(self, x, y):
        """Calculate Euclidean distance to another point"""
        return ((self.x - x) ** 2 + (self.y - y) ** 2) ** 0.5

    def __repr__(self):
        return f"{self.x}, {self.y}"

class Position:
    def __init__(self, x: float = 0, y: float = 0, theta: float = 0, speed: float = 0):
        self.x = x
        self.y = y
        self.theta = theta
        self.speed = speed
        self.left_inc = 0
        self.right_inc = 0

    def reset(self, x, y, theta, speed=0):
        self.x = x
        self.y = y
        self.theta = theta
        self.speed = speed

    def __repr__(self):
        return f"{self.x}, {self.y}"
    
class Area:

    YELLOW_HOME = ElementPosition(150 +225, 2000-225,  1.57)
    YELLOW_4    = ElementPosition(550 +225,       75, -1.57)
    YELLOW_2    = ElementPosition(1000+225,      225, -1.57)
    YELLOW_5    = ElementPosition(3000-225,       75, -1.57)
    YELLOW_3    = ElementPosition(3000-225, 1100-225,     0)

    BLUE_HOME  = ElementPosition(2850-225, 2000-225,  1.57)
    BLUE_5     = ElementPosition(     225,       75, -1.57)
    BLUE_3     = ElementPosition(     225, 1100-225,  3.14)
    BLUE_4     = ElementPosition(2000+225,       75, -1.57)
    BLUE_2     = ElementPosition(2000-225,      225, -1.57)

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
    STACK1  = ElementPosition(2175, 1725,  1.57)
    STACK2  = ElementPosition( 825, 1725,  1.57)

    STACK3  = ElementPosition(  75, 1325,  3.14)
    STACK4  = ElementPosition(  75,  400,  3.14)
    STACK5  = ElementPosition( 775,  250, -1.57)
    STACK9  = ElementPosition(1100,  950,  1.57)

    STACK8  = ElementPosition( 3000-75, 1325,     0)
    STACK7  = ElementPosition( 3000-75,  400,     0)
    STACK6  = ElementPosition(3000-775,  250, -1.57)
    STACK10 = ElementPosition(    1900,  950,  1.57)

    @classmethod
    def get_all_stacks(cls):
        """Get all stacks regardless of visited status"""
        return [(name, value) for name, value in cls.__dict__.items() 
               if isinstance(value, ElementPosition)]
    
    @classmethod
    def get_all_stacks(cls):
        """Get all stacks regardless of visited status"""
        return [(name, value) for name, value in cls.__dict__.items() 
               if isinstance(value, ElementPosition)]
    
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