import rclpy
from rclpy.node import Node
from crewai import Agent
from pydantic import BaseModel, Field
from campus_tour_interfaces.msg import VisitorRequest

class VisitorAgent(Agent, BaseModel):
    class Config:
        arbitrary_types_allowed = True

    name: str
    host: str
    request_pub: Node = Field(default=None)  # Make request_pub optional with a default value

    def __init__(self, config, node: Node):
        super().__init__(**config)
        self.host = config['host']
        self.request_pub = node.create_publisher(VisitorRequest, '/visitor_requests', 10)  # Initialize request_pub

    def request_escort(self, ci_agent):
        rclpy.loginfo(f"{self.name} requesting escort from {ci_agent.name} to meet {self.host}")

        if "CSE Department" in self.host:
            destination = "CSE Dept"
        elif "AI Department" in self.host:
            destination = "AI Dept"
        elif "Office" in self.host:
            destination = "Director's Office"
        elif "Cafeteria" in self.host:
            destination = "Cafeteria"
        else:
            raise ValueError(f"Invalid host format: {self.host}")

        valid_destinations = ["CSE Dept", "AI Dept", "Cafeteria", "Director's Office"]
        if destination not in valid_destinations:
            raise ValueError(f"Invalid destination: {destination}. Please check the host.")

        req = VisitorRequest()
        req.visitor_name = self.name
        req.destination = destination
        self.request_pub.publish(req)

        return ci_agent.escort_visitor(self, destination)
