import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation
import numpy as np
from robot_pkg.play_elements import Area, MaterialStack
from robot_pkg.move import Move, Opponent
from robot_pkg.lidar import Lidar

class FieldVisualizer:
    def __init__(self):
        self.fig, self.ax = plt.subplots(figsize=(12, 8))
        self.field_length = 3000  # mm
        self.field_width = 2000   # mm
        self.robot_pos = None
        self.opponent_pos = None
        self.robot_marker = None
        self.opponent_marker = None
        self.robot_arrow = None
        self.opponent_arrow = None
        self.robot_trail = []
        self.opponent_trail = []

        self.current_target = None
        
        self._setup_field()
        self.animation = FuncAnimation(self.fig, self.update_animation, interval=100)
        self.opponent = Opponent() # DELETE THIS
        
    def _setup_field(self):
        self.ax.set_xlim(0, self.field_length)
        self.ax.set_ylim(0, self.field_width)
        self.ax.set_aspect('equal')
        self.ax.grid(True)
        self.ax.set_title('Robot Table')
        self.ax.set_xlabel('X position (mm)')
        self.ax.set_ylabel('Y position (mm)')
        
        field = patches.Rectangle((0, 0), self.field_length, self.field_width, 
                                 linewidth=2, edgecolor='black', facecolor='gray', alpha=0.3)
        self.ax.add_patch(field)
        self._draw_areas()
        self._draw_stacks()
        
    def _draw_areas(self):
        for name, area in Area.get_all_areas():
            # Determine color and style based on availability
            if area.visited:
                color = 'lightyellow' if 'YELLOW' in name else 'lightblue'
                alpha = 0.2
                mark_visited = True
            else:
                color = 'yellow' if 'YELLOW' in name else 'blue'
                alpha = 0.5
                mark_visited = False
            
            # Set dimensions
            if '4' in name or '5' in name:
                width, height = 450, 150
            else:
                width, height = 450, 450
            
            # Draw rectangle
            x, y = area.x - width/2, area.y - height/2
            rect = patches.Rectangle((x, y), width, height, linewidth=2, edgecolor='red' if self.current_target == area else 'black',
                                  facecolor=color, alpha=alpha)
            self.ax.add_patch(rect)
            
            # Add text and visited mark
            self.ax.text(area.x, area.y, name, ha='center', va='center', fontsize=8)
            if mark_visited:
                self.ax.text(area.x, area.y, "✗", ha='center', va='center', 
                            fontsize=24, color='red', alpha=0.7)
                
    def set_current_target(self, target):
        """Set the current target (area or stack) for highlighting"""
        self.current_target = target
        self._setup_field()  # Redraw to update highlights

    def _draw_stacks(self):
        for name, stack in MaterialStack.get_all_stacks():  # Need to add this method
            # Determine color and style
            if stack.visited:
                color = 'sandybrown'
                alpha = 0.2
                mark_visited = True
            else:
                color = 'sienna'
                alpha = 0.7
                mark_visited = False
            
            # Draw stack
            rect = patches.Rectangle((stack.x-225, stack.y-50), 450, 100, rotation_point='center', angle=stack.theta*180/np.pi+90,
                                  facecolor=color, edgecolor='black', alpha=alpha)
            self.ax.add_patch(rect)
            
            # Add text and visited mark
            self.ax.text(stack.x, stack.y, name, ha='center', va='center', fontsize=8)
            if mark_visited:
                self.ax.text(stack.x, stack.y, "✗", ha='center', va='center',
                            fontsize=20, color='red', alpha=0.7)
    
    def update_animation(self, frame):
        #robot_x, robot_y = 1500 + 100 * np.sin(frame/10), 1000 + 100 * np.cos(frame/10)
        robot_x, robot_y = 1000,1500
        robot_theta = np.pi/4 + frame/20
        
        # Make opponent move between different areas
        #opponent_x = 2000 + 100 * np.sin(frame/15)
        #opponent_y = 1500 + 100 * np.cos(frame/15)
        opponent_x, opponent_y = 1000,1500
        opponent_theta = -np.pi/4 - frame/25
        
        self.update_positions(robot_x, robot_y, robot_theta, opponent_x, opponent_y, opponent_theta)
        self._draw_stacks()
        self._draw_areas()
    
    def update_positions(self, robot_x, robot_y, robot_theta, 
                        opponent_x=None, opponent_y=None, opponent_theta=None):
        if self.robot_marker:
            self.robot_marker.remove()
        if self.opponent_marker:
            self.opponent_marker.remove()
        if self.robot_arrow:
            self.robot_arrow.remove()
        if self.opponent_arrow:
            self.opponent_arrow.remove()
        
        self.robot_marker = patches.Circle((robot_x, robot_y), 190, 
                                         facecolor='blue', edgecolor='black')
        self.ax.add_patch(self.robot_marker)
        
        arrow_length = 200
        dx = arrow_length * np.cos(robot_theta)
        dy = arrow_length * np.sin(robot_theta)
        self.robot_arrow = self.ax.arrow(robot_x, robot_y, dx, dy, 
                                       head_width=80, head_length=100, fc='blue', ec='blue')
        
        if opponent_x is not None and opponent_y is not None:
            self.opponent_marker = patches.Circle((opponent_x, opponent_y), 190, 
                                                 facecolor='red', edgecolor='black')
            self.ax.add_patch(self.opponent_marker)
            self.opponent.update_position(opponent_x, opponent_y, # DELETE THIS LATER
                                    opponent_theta if opponent_theta is not None else 0, 
                                    0)
            
            if opponent_theta is not None:
                dx = arrow_length * np.cos(opponent_theta)
                dy = arrow_length * np.sin(opponent_theta)
                self.opponent_arrow = self.ax.arrow(opponent_x, opponent_y, dx, dy, 
                                                  head_width=80, head_length=100, fc='red', ec='red')
        
        if len(self.robot_trail) > 100:
            self.robot_trail.pop(0)
        self.robot_trail.append((robot_x, robot_y))
        self.ax.plot(*zip(*self.robot_trail), 'b-', alpha=0.3)

        if opponent_x is not None:
            if len(self.opponent_trail) > 100:
                self.opponent_trail.pop(0)
            self.opponent_trail.append((opponent_x, opponent_y))
            self.ax.plot(*zip(*self.opponent_trail), 'r-', alpha=0.3)
        
        self.fig.canvas.draw()
    
    def show(self):
        plt.tight_layout()
        plt.show()

if __name__ == "__main__":
    visualizer = FieldVisualizer()
    visualizer.show()