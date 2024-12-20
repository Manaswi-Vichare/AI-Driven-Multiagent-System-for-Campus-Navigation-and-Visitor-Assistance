import rclpy
from crewai import Agent
from pydantic import BaseModel
from pydantic import Field
from utils.campus_map import CampusMap
from campus_tour_interfaces.msg import NavReq, NavRes, PerformanceMetrics  # Import PerformanceMetrics
from campus_tour_interfaces.srv import Auth
from typing import Optional, List

class CampusInchargeAgent(Agent, BaseModel):
    name: str
    role: str
    goal: str
    backstory: str
    verbose: bool
    allow_delegation: bool
    tools: list
    campus_map: CampusMap = None
    visitors_entertained: int = 0  # Add metric for visitors
    violations: int = 0  # Add metric for violations
    nav_pub: Optional[rclpy.publisher.Publisher] = Field(default=None, exclude=True)  
    nav_sub: Optional[rclpy.subscription.Subscription] = Field(default=None, exclude=True)  
    performance_pub: Optional[rclpy.publisher.Publisher] = Field(default=None, exclude=True)  # Publisher for performance metrics

    class Config:
        arbitrary_types_allowed = True

    def __init__(self, config, node): 
        super().__init__(**config)
        self.campus_map = CampusMap()
        self.nav_pub = node.create_publisher(NavReq, '/navigation_requests', 10)
        self.nav_sub = node.create_subscription(NavRes, '/navigation_responses', self.handle_navigation_response, 10)
        self.performance_pub = node.create_publisher(PerformanceMetrics, '/agent_performance', 10)  # Initialize performance publisher

    def escort_visitor(self, visitor, destination):
        path = self.campus_map.get_path(self.campus_map.ENTRANCE, destination)
        rclpy.logging.get_logger('ci_agent').info(f"{self.name} is escorting {visitor.name} to {destination} via {path}")
        self.visitors_entertained += 1  # Increment the number of visitors entertained
        self.publish_metrics()  # Publish metrics after escorting the visitor
        return path

    def request_building_navigation(self, building_agent, visitor):
        req = NavReq()
        req.visitor_id = visitor.name
        req.building = building_agent.building
        self.nav_pub.publish(req)
        rclpy.wait_for_message('/navigation_responses', NavRes)

    def handle_navigation_response(self, msg):
        if msg.authorized:
            rclpy.logging.get_logger('ci_agent').info(f"{self.name} received navigation instructions for {msg.visitor_id} in {msg.building}")
            return msg.path
        else:
            rclpy.logging.get_logger('ci_agent').info(f"{self.name} was denied access for {msg.visitor_id} in {msg.building}")
            self.violations += 1  # Increment violations if access is denied
            self.publish_metrics()  # Publish metrics if a violation occurs
            return None

    def perform_task(self, task):
        rclpy.logging.get_logger('ci_agent').info(f"{self.name} is performing task: {task.description}")

    def publish_metrics(self):
        """Function to publish the agent's performance metrics."""
        msg = PerformanceMetrics()
        msg.agent_name = self.name
        msg.agent_type = 'CI'
        msg.visitors_entertained = self.visitors_entertained
        msg.violations = self.violations
        self.performance_pub.publish(msg)




# import rclpy
# from crewai import Agent
# from pydantic import BaseModel
# from pydantic import Field
# from utils.campus_map import CampusMap
# from campus_tour_interfaces.msg import NavReq, NavRes
# from campus_tour_interfaces.srv import Auth
# from typing import Optional, List

# class CampusInchargeAgent(Agent, BaseModel):
#     name: str
#     role: str
#     goal: str
#     backstory: str
#     verbose: bool
#     allow_delegation: bool
#     tools: list
#     campus_map: CampusMap = None
#     nav_pub: Optional[rclpy.publisher.Publisher] = Field(default=None, exclude=True)  
#     nav_sub: Optional[rclpy.subscription.Subscription] = Field(default=None, exclude=True)  

#     class Config:
#         arbitrary_types_allowed = True

#     def __init__(self, config, node): 
#         super().__init__(**config)
#         self.campus_map = CampusMap()
#         self.nav_pub = node.create_publisher(NavReq, '/navigation_requests', 10)  
#         self.nav_sub = node.create_subscription(NavRes, '/navigation_responses', self.handle_navigation_response, 10)  

#     def escort_visitor(self, visitor, destination):
#         path = self.campus_map.get_path(self.campus_map.ENTRANCE, destination)
#         rclpy.logging.get_logger('ci_agent').info(f"{self.name} is escorting {visitor.name} to {destination} via {path}")
#         return path

#     def request_building_navigation(self, building_agent, visitor):
#         req = NavReq()
#         req.visitor_id = visitor.name
#         req.building = building_agent.building
#         self.nav_pub.publish(req)
#         rclpy.wait_for_message('/navigation_responses', NavRes)

#     def handle_navigation_response(self, msg):
#         if msg.authorized:
#             rclpy.logging.get_logger('ci_agent').info(f"{self.name} received navigation instructions for {msg.visitor_id} in {msg.building}")
#             return msg.path
#         else:
#             rclpy.logging.get_logger('ci_agent').info(f"{self.name} was denied access for {msg.visitor_id} in {msg.building}")
#             return None

#     def perform_task(self, task):
#         rclpy.logging.get_logger('ci_agent').info(f"{self.name} is performing task: {task.description}")

    
