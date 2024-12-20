#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from agents.CI_agents import CampusInchargeAgent
import yaml

class CIAgentNode(Node):
    def __init__(self):
        super().__init__('ci_agent_node')
        self.config_path = self.declare_parameter('config_path', 'src/campus_tour/campus_tour/config/CI_agents.yaml').value
        ci_config = self.load_config(self.config_path)
        self.ci_agent = CampusInchargeAgent(ci_config, self)  # Pass the node to the agent

    def load_config(self, file_path):
        with open(file_path, 'r') as file:
            return yaml.safe_load(file)

def main(args=None):
    rclpy.init(args=args)  
    ci_agent_node = CIAgentNode()  
    rclpy.spin(ci_agent_node)  

    ci_agent_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
