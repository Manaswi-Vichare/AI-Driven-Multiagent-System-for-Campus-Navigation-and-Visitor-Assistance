import rclpy
from rclpy.node import Node
import networkx as nx
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.animation import FuncAnimation
from matplotlib.collections import LineCollection
import numpy as np

class CampusMap:
    ENTRANCE = "Entrance"

    def __init__(self):
        self.G = nx.Graph()
        self.locations = ["Entrance", "CSE Dept", "AI Dept", "Cafeteria", "Director Office"]
        self.G.add_nodes_from(self.locations)
        edges = [
            ("Entrance", "CSE Dept", 5),
            ("Entrance", "AI Dept", 6),
            ("Entrance", "Director Office", 8),
            ("CSE Dept", "Cafeteria", 3),
            ("AI Dept", "Cafeteria", 4),
            ("CSE Dept", "Director Office", 4),
            ("AI Dept", "Director Office", 3),
        ]
        self.G.add_weighted_edges_from(edges)
        self.visitor_paths = {}

    def add_visitor_path(self, visitor_name, start, end):
        shortest_path = nx.shortest_path(self.G, source=start, target=end, weight='weight')
        self.visitor_paths[visitor_name] = shortest_path

    def animate_paths(self, output_gif='vis3.gif'):
        fig, ax = plt.subplots(figsize=(11, 6))
        pos = nx.spring_layout(self.G, seed=3)

        def draw_base_map():
            nx.draw_networkx_nodes(self.G, pos, node_size=2500, node_color='skyblue', alpha=0.9, ax=ax)
            nx.draw_networkx_edges(self.G, pos, width=2, alpha=0.7, edge_color='gray', ax=ax)
            nx.draw_networkx_labels(self.G, pos, font_size=12, font_color='black', font_weight='bold', ax=ax)
            edge_labels = nx.get_edge_attributes(self.G, 'weight')
            nx.draw_networkx_edge_labels(self.G, pos, edge_labels=edge_labels, ax=ax)

        def update_node_colors(agent_position, destination, status):
            color_mapping = {
                'default': 'skyblue',
                'travelling': 'yellow',
                'arrived': 'green',
                'denied': 'red',
            }

            colors = []
            for loc in self.locations:
                if loc == agent_position and status == 'travelling':
                    colors.append(color_mapping['travelling'])
                elif loc == destination:
                    colors.append(color_mapping[status])  # Use the status directly for destination
                else:
                    colors.append(color_mapping['default'])

            nx.draw_networkx_nodes(self.G, pos, node_size=2500, node_color=colors, alpha=0.9, ax=ax)

        def update(frame):
            ax.clear()
            draw_base_map()

            for visitor, path in self.visitor_paths.items():
                path_length = len(path)
                formatted_path = ' -> '.join(path)  # Format the path as "Entrance -> Destination"
                title_text = f"{formatted_path} ({visitor})"  # Show path and visitor's name in title

                total_distance = 0  # Initialize total distance for the visitor

                if frame < (path_length - 1) * 10:  # Scale frames for each path segment
                    segment = frame // 10  # Each segment takes 10 frames
                    progress = (frame % 10) / 10.0  # Progress within the segment (0 to 1)

                    if segment < path_length - 1:
                        current = path[segment]
                        next_location = path[segment + 1]

                        # Calculate distance between current and next node
                        distance = self.G[current][next_location]['weight']
                        total_distance = sum(self.G[path[i]][path[i + 1]]['weight'] for i in range(segment)) + distance * progress

                        # Log traversal details
                        print(f"{visitor} is traveling from {current} to {next_location}. "
                            f"Segment Distance: {distance} units. "
                            f"Total Distance so far: {total_distance:.2f} units.")

                        # Update only the current node as travelling (yellow)
                        update_node_colors(current, next_location, 'travelling')

                        # Calculate the positions for the animated line
                        start_pos = pos[current]
                        end_pos = pos[next_location]

                        current_pos = (
                            start_pos[0] + progress * (end_pos[0] - start_pos[0]),
                            start_pos[1] + progress * (end_pos[1] - start_pos[1])
                        )

                        # Draw the moving line
                        line = LineCollection([(start_pos, current_pos)], colors=['red'], linewidths=3, linestyles='dashed')
                        ax.add_collection(line)

                else:  # Final node reached
                    final_status = 'denied' if visitor == 'Visitor 5' else 'arrived'
                    update_node_colors(path[-1], path[-1], final_status)

                    # Calculate total distance for the entire journey
                    total_distance = sum(self.G[path[i]][path[i + 1]]['weight'] for i in range(path_length - 1))
                    print(f"{visitor} has reached {path[-1]}. Total Distance: {total_distance:.2f} units.")

                plt.title(title_text, fontsize=16)  # Update title with path and visitor's name

            plt.axis('off')
        # Calculate total frames needed for smoother traversal
        total_frames = sum((len(path) - 1) * 10 for path in self.visitor_paths.values()) + 1  # Increase for smoothness
        print(f"Total frames for animation: {total_frames}")

        ani = FuncAnimation(fig, update, frames=total_frames, repeat=False)  # Use the new total frame count
        ani.save(output_gif, writer='pillow', fps=3)  # Decreased fps for slower animation
        print(f"Animation saved as: {output_gif}")

# Example main function to run the simulation
def main(args=None):
    rclpy.init(args=args)
    campus_map = CampusMap()

    # campus_map.add_visitor_path('Visitor 1', CampusMap.ENTRANCE, 'CSE Dept')
    # campus_map.add_visitor_path('Visitor 2', CampusMap.ENTRANCE, 'AI Dept')
    campus_map.add_visitor_path('Visitor 3', CampusMap.ENTRANCE, 'Cafeteria')

    # campus_map.add_visitor_path('Visitor 5', CampusMap.ENTRANCE, 'Director Office')  # Denied access case

    try:
        campus_map.animate_paths()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
     main()



