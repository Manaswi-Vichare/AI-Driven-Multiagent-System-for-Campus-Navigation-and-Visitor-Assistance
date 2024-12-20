import rclpy
import time
from rclpy.node import Node
from crewai import Agent
from pydantic import BaseModel, Field
from campus_tour_interfaces.msg import NavReq, NavRes, PerformanceMetrics  # Import PerformanceMetrics
from campus_tour_interfaces.srv import Auth, OOS
from typing import Any

class BuildingInchargeAgent(Agent, BaseModel):
    name: str
    building: str 
    out_of_service: bool = False
    oos_duration: int = 0
    oos_start_time: float = 0
    visitor_queue: list = []
    current_visitor: dict = None
    visit_time_limit: int = 600  # 10 minutes in seconds
    visitors_entertained: int = 0  # Metric for visitors entertained
    violations: int = 0  # Metric for violations

    auth_service: Any = None
    oos_service: Any = None
    nav_sub: Any = None
    nav_pub: Any = None
    performance_pub: Any = None  # Publisher for performance metrics

    class Config:
        arbitrary_types_allowed = True

    def __init__(self, config, node: Node):
        super().__init__(**config)
        self.building = config['building'].replace(" ", "_")  # Sanitize building name

        self.nav_pub = node.create_publisher(NavRes, '/navigation_responses', 10)
        self.nav_sub = node.create_subscription(NavReq, '/navigation_requests', self.handle_navigation_request, 10)

        # Services for authorization and OOS handling
        self.auth_service = node.create_service(Auth, f'/{self.building}/authorize', self.handle_authorization)
        self.oos_service = node.create_service(OOS, f'/{self.building}/oos', self.handle_oos)

        self.performance_pub = node.create_publisher(PerformanceMetrics, '/agent_performance', 10)  # Initialize performance publisher

    def handle_navigation_request(self, req):
        if self.out_of_service:
            rclpy.logging.get_logger(self.name).info(f"{self.name} is out of service. Cannot provide navigation.")
            self.send_navigation_response(req.visitor_id, False, None)
        elif self.check_authorization(req.visitor_id):
            path = f"Path within {self.building} for {req.visitor_id}"
            rclpy.logging.get_logger(self.name).info(f"{self.name} providing navigation in {self.building} for {req.visitor_id}")
            self.visitors_entertained += 1  # Increment entertained visitors
            self.publish_metrics()  # Publish metrics
            self.send_navigation_response(req.visitor_id, True, path)
        else:
            rclpy.logging.get_logger(self.name).info(f"{self.name} denying access to {req.visitor_id} in {self.building}")
            self.send_navigation_response(req.visitor_id, False, None)

    def send_navigation_response(self, visitor_id, authorized, path):
        resp = NavRes()
        resp.visitor_id = visitor_id
        resp.building = self.building
        resp.authorized = authorized
        resp.path = path if path else ""
        self.nav_pub.publish(resp)

    def handle_authorization(self, req):
        return self.check_authorization(req.visitor_id)

    def handle_oos(self, req):
        self.go_out_of_service(req.duration)
        return True

    def go_out_of_service(self, duration):
        self.out_of_service = True
        self.oos_duration = duration
        self.oos_start_time = time.time()
        rclpy.logging.get_logger(self.name).info(f"{self.name} is going out of service for {duration} seconds")

    def check_out_of_service(self):
        if self.out_of_service:
            elapsed_time = time.time() - self.oos_start_time
            if elapsed_time > self.oos_duration:
                self.out_of_service = False
                rclpy.logging.get_logger(self.name).info(f"{self.name} is back in service")
                if elapsed_time > self.oos_duration + 60:  # 1 minute grace period
                    rclpy.logging.get_logger(self.name).warn(f"Violation: {self.name} exceeded OOS duration by {elapsed_time - self.oos_duration} seconds")
                    self.violations += 1  # Increment violations for exceeding OOS duration
                    self.publish_metrics()  # Publish metrics when a violation occurs
            return self.out_of_service
        return False

    def check_authorization(self, visitor_id):
        # Simulating authorization check
        return visitor_id.split()[-1] == self.building

    def add_to_queue(self, visitor):
        self.visitor_queue.append(visitor)
        rclpy.logging.get_logger(self.name).info(f"{visitor} added to queue for {self.building}")

    def process_queue(self):
        if self.current_visitor:
            visit_duration = time.time() - self.current_visitor['start_time']
            if visit_duration > self.visit_time_limit:
                rclpy.logging.get_logger(self.name).info(f"{self.current_visitor['visitor']}'s time is up in {self.building}")
                self.current_visitor = None

        if not self.current_visitor and self.visitor_queue:
            next_visitor = self.visitor_queue.pop(0)
            self.current_visitor = {'visitor': next_visitor, 'start_time': time.time()}
            rclpy.logging.get_logger(self.name).info(f"{next_visitor} is now meeting in {self.building}")

    def perform_task(self, task):
        rclpy.logging.get_logger(self.name).info(f"{self.name} is performing task: {task.description}")

    def publish_metrics(self):
        """Function to publish the agent's performance metrics."""
        msg = PerformanceMetrics()
        msg.agent_name = self.name
        msg.agent_type = 'BI'
        msg.visitors_entertained = self.visitors_entertained
        msg.violations = self.violations
        self.performance_pub.publish(msg)


# import rclpy
# import time
# from rclpy.node import Node  # Import Node from rclpy
# from crewai import Agent
# from pydantic import BaseModel, Field  # Import Field from pydantic
# from campus_tour_interfaces.msg import NavReq, NavRes
# from campus_tour_interfaces.srv import Auth, OOS
# from typing import Any  # Import Optional from typing

# class BuildingInchargeAgent(Agent, BaseModel):
#     name: str
#     building: str 
#     out_of_service: bool = False
#     oos_duration: int = 0
#     oos_start_time: float = 0
#     visitor_queue: list = []
#     current_visitor: dict = None
#     visit_time_limit: int = 600  # 10 minutes in seconds

#     # Define services and subscribers as class attributes, not Pydantic fields
#     auth_service: Any = None
#     oos_service: Any = None
#     nav_sub: Any = None
#     nav_pub: Any = None

#     class Config:
#         arbitrary_types_allowed = True

#     def __init__(self, config, node: Node):
#         super().__init__(**config)
#         self.building = config['building'].replace(" ", "_")  # Sanitize building name
#         # rclpy.init_node(f'bi_agent_{self.name}', anonymous=True)

#         # Create the publisher and subscriber using the node instance
#         self.nav_pub = node.create_publisher(NavRes, '/navigation_responses', 10)
#         self.nav_sub = node.create_subscription(NavReq, '/navigation_requests', self.handle_navigation_request, 10)

#         # Services for authorization and OOS handling
#         self.auth_service = node.create_service(Auth, f'/{self.building}/authorize', self.handle_authorization)
#         self.oos_service = node.create_service(OOS, f'/{self.building}/oos', self.handle_oos)

#     def handle_navigation_request(self, req):
#         if self.out_of_service:
#             rclpy.logging.get_logger(self.name).info(f"{self.name} is out of service. Cannot provide navigation.")
#             self.send_navigation_response(req.visitor_id, False, None)
#         elif self.check_authorization(req.visitor_id):
#             path = f"Path within {self.building} for {req.visitor_id}"
#             rclpy.logging.get_logger(self.name).info(f"{self.name} providing navigation in {self.building} for {req.visitor_id}")
#             self.send_navigation_response(req.visitor_id, True, path)
#         else:
#             rclpy.logging.get_logger(self.name).info(f"{self.name} denying access to {req.visitor_id} in {self.building}")
#             self.send_navigation_response(req.visitor_id, False, None)

#     def send_navigation_response(self, visitor_id, authorized, path):
#         resp = NavRes()
#         resp.visitor_id = visitor_id
#         resp.building = self.building
#         resp.authorized = authorized
#         resp.path = path if path else ""
#         self.nav_pub.publish(resp)

#     def handle_authorization(self, req):
#         return self.check_authorization(req.visitor_id)

#     def handle_oos(self, req):
#         self.go_out_of_service(req.duration)
#         return True

#     def go_out_of_service(self, duration):
#         self.out_of_service = True
#         self.oos_duration = duration
#         self.oos_start_time = time.time()
#         rclpy.logging.get_logger(self.name).info(f"{self.name} is going out of service for {duration} seconds")

#     def check_out_of_service(self):
#         if self.out_of_service:
#             elapsed_time = time.time() - self.oos_start_time
#             if elapsed_time > self.oos_duration:
#                 self.out_of_service = False
#                 rclpy.logging.get_logger(self.name).info(f"{self.name} is back in service")
#                 if elapsed_time > self.oos_duration + 60:  # 1 minute grace period
#                     rclpy.logging.get_logger(self.name).warn(f"Violation: {self.name} exceeded OOS duration by {elapsed_time - self.oos_duration} seconds")
#             return self.out_of_service
#         return False

#     def check_authorization(self, visitor_id):
#         # Simulating authorization check
#         return visitor_id.split()[-1] == self.building

#     def add_to_queue(self, visitor):
#         self.visitor_queue.append(visitor)
#         rclpy.logging.get_logger(self.name).info(f"{visitor} added to queue for {self.building}")

#     def process_queue(self):
#         if self.current_visitor:
#             visit_duration = time.time() - self.current_visitor['start_time']
#             if visit_duration > self.visit_time_limit:
#                 rclpy.logging.get_logger(self.name).info(f"{self.current_visitor['visitor']}'s time is up in {self.building}")
#                 self.current_visitor = None

#         if not self.current_visitor and self.visitor_queue:
#             next_visitor = self.visitor_queue.pop(0)
#             self.current_visitor = {'visitor': next_visitor, 'start_time': time.time()}
#             rclpy.logging.get_logger(self.name).info(f"{next_visitor} is now meeting in {self.building}")

#     def perform_task(self, task):
#         rclpy.logging.get_logger(self.name).info(f"{self.name} is performing task: {task.description}")
