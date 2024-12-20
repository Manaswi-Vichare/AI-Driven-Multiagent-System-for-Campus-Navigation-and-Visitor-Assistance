#!/usr/bin/env python3
import rclpy
from agents.Visitor_agents import VisitorAgent
import yaml

def load_config(file_path):
    with open(file_path, 'r') as file:
        return yaml.safe_load(file)

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('visitor_node')

    # Declare parameters with default values
    config_path = node.declare_parameter('config_path', 'src/campus_tour/campus_tour/config/Visitor_agents.yaml').value
    name = node.declare_parameter('name', 'Default Visitor').value
    host = node.declare_parameter('host', 'Default Host').value

    # Load the configuration file
    visitor_config = load_config(config_path)
    
    # Create the VisitorAgent instance, passing the loaded config and the node
    visitor_agent = VisitorAgent({**visitor_config, 'name': name, 'host': host}, node)

    try:
        rclpy.spin(node)  # Keep the node alive
    except rclpy.ROSInterruptException:
        pass
    finally:
        node.destroy_node()  # Properly destroy the node
        rclpy.shutdown()  # Shutdown the rclpy library

if __name__ == '__main__':
    main()
