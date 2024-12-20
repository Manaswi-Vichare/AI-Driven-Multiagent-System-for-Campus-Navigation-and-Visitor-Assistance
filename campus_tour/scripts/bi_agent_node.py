#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from agents.BI_agents import BuildingInchargeAgent
import yaml

class BIAGentNode(Node):
    def __init__(self):
        super().__init__('bi_agent_node')
        self.declare_parameter('config_path', 'src/campus_tour/campus_tour/config/BI_agents.yaml')
        self.declare_parameter('building', 'Default Building')

        # Get parameters
        config_path = self.get_parameter('config_path').get_parameter_value().string_value
        building = self.get_parameter('building').get_parameter_value().string_value

        # Load configuration and create BuildingInchargeAgent with Node instance
        bi_config = self.load_config(config_path)
        self.bi_agent = BuildingInchargeAgent({**bi_config, 'building': building}, self)  # Pass self as the node

    def load_config(self, file_path):
        with open(file_path, 'r') as file:
            return yaml.safe_load(file)

def main(args=None):
    rclpy.init(args=args)
    bi_agent_node = BIAGentNode()

    try:
        rclpy.spin(bi_agent_node)
    except rclpy.exceptions.ROSInterruptException:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
