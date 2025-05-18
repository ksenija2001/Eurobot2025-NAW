import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation
import numpy as np
import socket
import struct
from robot_pkg.play_elements import Area, MaterialStack, Position, ElementPosition
from threading import Thread

class FieldVisualizer:
    def __init__(self, w, h):
        self.fig, self.ax = plt.subplots(figsize=(12, 8))
        self.field_length = w  # mm
        self.field_width = h   # mm
        self.robot_pos: Position = Position()
        self.opponent_pos: Position = Position()

        self.robot_marker = None
        self.opponent_marker = None
        self.robot_arrow = None
        self.opponent_arrow = None
        self.robot_trail = []
        self.opponent_trail = []

        self.pc_socket = None
        self.connection = None

        self._setup_connection()
        self.running = False
        self.con_thread = Thread(target=self.receive_opponent_info)

        # Setup the field
        self._setup_field()

        self.animation = FuncAnimation(
            self.fig, self.update_positions, interval=100)

    def receive_opponent_info(self):
        while self.running:
            try:
                
                msg = self.connection.recv(64)

                # print(f"Received: {msg}")
                key = str(msg[0])
                print(f"Key: {key}")
                if key == '79':
                    print(f"Opponent")
                    pose = struct.unpack('3f', msg[1:])
                    self.opponent_pos.x = pose[0]
                    self.opponent_pos.y = pose[1]
                    self.opponent_pos.theta = pose[2]
                elif key == '82':
                    print(f"Robot")
                    pose = struct.unpack('3f', msg[1:])
                    self.robot_pos.x = pose[0]
                    self.robot_pos.y = pose[1]
                    self.robot_pos.theta = pose[2]
                elif key == '83':
                    length = int(msg[1])
                    data = str(msg[1:length+2].decode('utf-8'))
                    print(f"Stack: {data}")
                    for name, stack in vars(MaterialStack).items():
                        print(f"Stack name: {name}")
                        if name in data:
                            if '10' in data:
                                print(f"Stack value: {MaterialStack.STACK10}")
                                MaterialStack.STACK10.visited = True
                            else:
                                print(f"Stack value: {stack}")
                                stack.visited = True
                            self._draw_stacks()
                            break
                elif key == '65':
                    length = int(msg[1])
                    data = str(msg[1:length+2].decode('utf-8'))
                    print(f"Area: {data}")
                    for name, area in vars(Area).items():
                        if name in data:
                            print(f"Area value: {area}")
                            area.visited = True
                            self._draw_areas()
                            # print(name, area.visited)
                            break
                        # print(name, area.visited)
            except Exception as e:
                print(e)

    def _setup_connection(self):
        print(f"Started connecting...")
        try:
            self.pc_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.pc_socket.setsockopt(
                socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.pc_socket.bind(('10.166.197.207', 9999))

            self.pc_socket.listen(1)

            self.connection, address = self.pc_socket.accept()

            print("Computer connected")
        except Exception as e:
            print(e)

    def _setup_field(self):
        """
            Initialize the field visualization
        """

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
        """
            Draw all defined areas
        """
        for name, area in Area.get_all_areas():
            # Determine color and style based on availability
            if area.visited:
                color = 'lightyellow' if 'YELLOW' in name else 'lightblue'
                alpha = 0.2
                mark_visited = True
            else:
                color = 'yellow' if 'YELLOW' in name else 'blue'
                alpha = 0.2
                mark_visited = False
            
            if '4' in name or '5' in name:
                width, height = 450, 150
            else:
                width, height = 450, 450
            
            x, y = area.x - width/2, area.y - height/2
            rect = patches.Rectangle((x, y), width, height, linewidth=2, edgecolor='black',
                                  facecolor=color, alpha=alpha)
            self.ax.add_patch(rect)
            
            self.ax.text(area.x, area.y, name, ha='center', va='center', fontsize=8)
            if mark_visited:
                self.ax.text(area.x, area.y, "✗", ha='center', va='center', 
                            fontsize=24, color='red', alpha=0.7)

    def _draw_stacks(self):
        """Draw all material stacks"""
        for name, stack in MaterialStack.get_all_stacks():
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

    # def _update_animation(self, frame):
    #     # Get latest positions from Move.pose and Lidar.opponent
    #     self.update_positions(
    #         robot_x=Move.pose.x,
    #         robot_y=Move.pose.y,
    #         robot_theta=Move.pose.theta,
    #         opponent_x=Lidar.opponent.x,
    #         opponent_y=Lidar.opponent.y,
    #         opponent_theta=Lidar.opponent.theta
    #     )

    def update_positions(self, t):
        """Update the positions of the robot and opponent"""
        # Clear previous markers if they exist
        if self.robot_pos is None or self.opponent_pos is None:
            return

        if self.robot_marker:
            self.robot_marker.remove()
        if self.opponent_marker:
            self.opponent_marker.remove()
        if self.robot_arrow:
            self.robot_arrow.remove()
        if self.opponent_arrow:
            self.opponent_arrow.remove()

        # Update robot position
        self.robot_marker = patches.Circle((self.robot_pos.x, self.robot_pos.y), 100,
                                           facecolor='blue', edgecolor='black')
        self.ax.add_patch(self.robot_marker)

        # Add direction arrow for robot
        arrow_length = 200
        dx = arrow_length * np.cos(self.robot_pos.theta)
        dy = arrow_length * np.sin(self.robot_pos.theta)
        self.robot_arrow = self.ax.arrow(self.robot_pos.x, self.robot_pos.y, dx, dy,
                                         head_width=80, head_length=100, fc='blue', ec='blue')

        if len(self.robot_trail) > 100:  # Limit trail length
            self.robot_trail.pop(0)
        self.robot_trail.append((self.robot_pos.x, self.robot_pos.y))
        self.ax.plot(*zip(*self.robot_trail), 'b-', alpha=0.3)  # Blue trail

        self.ax.text(self.robot_pos.x, self.robot_pos.y + 150, 'Robot',
                     ha='center', va='center', color='blue')

        # Update opponent position if provided
        self.opponent_marker = patches.Circle((self.opponent_pos.x, self.opponent_pos.y), 100,
                                              facecolor='red', edgecolor='black')
        self.ax.add_patch(self.opponent_marker)

        # Add direction arrow for opponent
        if self.opponent_pos.theta != 0:
            dx = arrow_length * np.cos(self.opponent_pos.theta)
            dy = arrow_length * np.sin(self.opponent_pos.theta)
            self.opponent_arrow = self.ax.arrow(self.opponent_pos.x, self.opponent_pos.y, dx, dy,
                                                head_width=80, head_length=100, fc='red', ec='red')

        if len(self.opponent_trail) > 100:
            self.opponent_trail.pop(0)

        self.opponent_trail.append((self.opponent_pos.x, self.opponent_pos.y))
        self.ax.plot(*zip(*self.opponent_trail),
                     'r-', alpha=0.3)  # Red trail

        self.ax.text(self.opponent_pos.x, self.opponent_pos.y + 150, 'Opponent',
                     ha='center', va='center', color='red')

        self.fig.canvas.draw()

    def show(self):
        """Display the field visualization"""
        plt.tight_layout()
        plt.show()


# Example usage:
if __name__ == "__main__":
    # Create visualizer
    visualizer = FieldVisualizer(3000, 2000)
    visualizer.running = True
    visualizer.con_thread.start()

    # # Example positions (replace with actual data from your system)
    # robot_x, robot_y, robot_theta = 1500, 1000, np.pi/4  # 45 degrees
    # opponent_x, opponent_y, opponent_theta = 2000, 1500, -np.pi/4  # -45 degrees

    # # Update positions
    # visualizer.update_positions(robot_x, robot_y, robot_theta,
    #                             opponent_x, opponent_y, opponent_theta)

    # Show the visualization
    visualizer.show()
