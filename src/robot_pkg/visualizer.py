import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation
import numpy as np
from robot_pkg.play_elements import Area, MaterialStack
from robot_pkg.move import Move
from robot_pkg.lidar import Lidar


class FieldVisualizer:
    def __init__(self, w, h):
        self.fig, self.ax = plt.subplots(figsize=(12, 8))
        self.field_length = w  # mm
        self.field_width = h   # mm
        self.robot_pos = None
        self.opponent_pos = None
        self.robot_marker = None
        self.opponent_marker = None
        self.robot_arrow = None
        self.opponent_arrow = None
        self.robot_trail = []
        self.opponent_trail = []

        # Setup the field
        self._setup_field()

        self.animation = FuncAnimation(
            self.fig, self.update_positions, interval=100)

    def _setup_field(self):
        """
            Initialize the field visualization
        """

        self.ax.set_xlim(0, self.field_length)
        self.ax.set_ylim(0, self.field_width)
        self.ax.set_aspect('equal')
        self.ax.grid(True)
        self.ax.set_title('Robot Field')
        self.ax.set_xlabel('X position (mm)')
        self.ax.set_ylabel('Y position (mm)')

        field = patches.Rectangle((0, 0), self.field_length, self.field_width,
                                  linewidth=2, edgecolor='black', facecolor='green', alpha=0.3)
        self.ax.add_patch(field)

        # Draw center line
        self.ax.plot([self.field_length/2, self.field_length/2], [0, self.field_width],
                     'white', linewidth=2, linestyle='--')

        # Draw areas
        self._draw_areas()

        # Draw stacks
        self._draw_stacks()

    def _draw_areas(self):
        """
            Draw all defined areas
        """

        areas = Area.get_all_areas()
        for name, area in areas:
            color = 'yellow' if 'YELLOW' in name else 'blue'
            circle = patches.Circle((area.x, area.y), 225,
                                    facecolor=color, edgecolor='black', alpha=0.5)
            self.ax.add_patch(circle)
            self.ax.text(area.x, area.y, str(area).split('.')[-1],
                         ha='center', va='center', fontsize=8)

    def _draw_stacks(self):
        """Draw all material stacks"""
        for stack in MaterialStack.get_all_unvisited_stacks():
            color = 'orange'
            circle = patches.Circle((stack.x, stack.y), 150,
                                    facecolor=color, edgecolor='black', alpha=0.7)
            self.ax.add_patch(circle)
            self.ax.text(stack.x, stack.y, str(stack).split('.')[-1],
                         ha='center', va='center', fontsize=8)

    def _update_animation(self, frame):
        # Get latest positions from Move.pose and Lidar.opponent
        self.update_positions(
            robot_x=Move.pose.x,
            robot_y=Move.pose.y,
            robot_theta=Move.pose.theta,
            opponent_x=Lidar.opponent.x,
            opponent_y=Lidar.opponent.y,
            opponent_theta=Lidar.opponent.theta
        )

    def update_positions(self, robot_x, robot_y, robot_theta,
                         opponent_x=None, opponent_y=None, opponent_theta=None):
        """Update the positions of the robot and opponent"""
        # Clear previous markers if they exist
        if self.robot_marker:
            self.robot_marker.remove()
        if self.opponent_marker:
            self.opponent_marker.remove()
        if self.robot_arrow:
            self.robot_arrow.remove()
        if self.opponent_arrow:
            self.opponent_arrow.remove()

        # Update robot position
        self.robot_marker = patches.Circle((robot_x, robot_y), 100,
                                           facecolor='blue', edgecolor='black')
        self.ax.add_patch(self.robot_marker)

        # Add direction arrow for robot
        arrow_length = 200
        dx = arrow_length * np.cos(robot_theta)
        dy = arrow_length * np.sin(robot_theta)
        self.robot_arrow = self.ax.arrow(robot_x, robot_y, dx, dy,
                                         head_width=80, head_length=100, fc='blue', ec='blue')

        if len(self.robot_trail) > 100:  # Limit trail length
            self.robot_trail.pop(0)
        self.robot_trail.append((robot_x, robot_y))
        self.ax.plot(*zip(*self.robot_trail), 'b-', alpha=0.3)  # Blue trail

        self.ax.text(robot_x, robot_y + 150, 'Robot',
                     ha='center', va='center', color='blue')

        # Update opponent position if provided
        if opponent_x != 0 and opponent_y != 0:
            self.opponent_marker = patches.Circle((opponent_x, opponent_y), 100,
                                                  facecolor='red', edgecolor='black')
            self.ax.add_patch(self.opponent_marker)

            # Add direction arrow for opponent
            if opponent_theta != 0:
                dx = arrow_length * np.cos(opponent_theta)
                dy = arrow_length * np.sin(opponent_theta)
                self.opponent_arrow = self.ax.arrow(opponent_x, opponent_y, dx, dy,
                                                    head_width=80, head_length=100, fc='red', ec='red')

            if len(self.opponent_trail) > 100:
                self.opponent_trail.pop(0)

            self.opponent_trail.append((opponent_x, opponent_y))
            self.ax.plot(*zip(*self.opponent_trail),
                         'r-', alpha=0.3)  # Red trail

            self.ax.text(opponent_x, opponent_y + 150, 'Opponent',
                         ha='center', va='center', color='red')

        self.fig.canvas.draw()

    def show(self):
        """Display the field visualization"""
        plt.tight_layout()
        plt.show()


# Example usage:
if __name__ == "__main__":
    # Create visualizer
    visualizer = FieldVisualizer()

    # Example positions (replace with actual data from your system)
    robot_x, robot_y, robot_theta = 1500, 1000, np.pi/4  # 45 degrees
    opponent_x, opponent_y, opponent_theta = 2000, 1500, -np.pi/4  # -45 degrees

    # Update positions
    visualizer.update_positions(robot_x, robot_y, robot_theta,
                                opponent_x, opponent_y, opponent_theta)

    # Show the visualization
    visualizer.show()
