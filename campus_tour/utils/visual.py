import rclpy
import matplotlib.pyplot as plt
from std_msgs.msg import String
from campus_tour_interfaces.msg import AgentPosition

class Visualizer:
    def __init__(self):
        self.campus_map = None
        self.agent_positions = {}
        self.node = rclpy.create_node('campus_visualizer')  # Create the node here
        self.node.create_subscription(String, '/campus_map', self.update_map, 10)
        self.node.create_subscription(AgentPosition, '/agent_positions', self.update_agent_position, 10)

    def update_map(self, msg):
        self.campus_map = eval(msg.data)  # Convert string representation to dictionary

    def update_agent_position(self, msg):
        self.agent_positions[msg.agent_name] = (msg.x, msg.y)

    def visualize(self):
        if self.campus_map is None:
            return

        plt.clf()
        for building, pos in self.campus_map.items():
            plt.plot(pos[0], pos[1], 'bo', markersize=10)
            plt.text(pos[0], pos[1], building, fontsize=8, ha='center', va='bottom')

        for agent, pos in self.agent_positions.items():
            plt.plot(pos[0], pos[1], 'ro', markersize=5)
            plt.text(pos[0], pos[1], agent, fontsize=6, ha='center', va='bottom')

        plt.title('Campus Tour Visualization')
        plt.axis('equal')
        plt.draw()
        plt.pause(0.1)

def visualize_campus():
    if not rclpy.ok():  # Ensure that rclpy is not initialized yet
        rclpy.init()  # Initialize rclpy only if not already initialized
    visualizer = Visualizer()  # Initialize the visualizer after rclpy.init()
    rate = visualizer.node.create_rate(10)  # 10 Hz
    while rclpy.ok():  # Check if rclpy is still running
        rclpy.spin_once(visualizer.node, timeout_sec=0.1)
        visualizer.visualize()
        rate.sleep()

if __name__ == '__main__':
    try:
        visualize_campus()
    except rclpy.ROSInterruptException:
        pass
