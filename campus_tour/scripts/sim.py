import rclpy
from rclpy.node import Node
from campus_tour_interfaces.msg import AgentPosition, NavReq, NavRes, VisitorRequest
from campus_tour_interfaces.srv import Auth, OOS
from utils.campus_map import CampusMap

class CampusTourSimulation(Node):
    def __init__(self):
        super().__init__('campus_tour_simulation')
        self.campus_map = CampusMap()
        
        # Initialize publishers
        self.agent_position_pub = self.create_publisher(AgentPosition, 'agent_position', 10)
        self.nav_req_pub = self.create_publisher(NavReq, 'navigation_request', 10)
        
        # Initialize subscribers
        self.nav_res_sub = self.create_subscription(NavRes, 'navigation_response', self.nav_response_callback, 10)
        self.visitor_req_sub = self.create_subscription(VisitorRequest, 'visitor_request', self.visitor_request_callback, 10)
        
        # Initialize service clients
        self.auth_client = self.create_client(Auth, 'authorization')
        self.oos_client = self.create_client(OOS, 'out_of_service')
        
        self.get_logger().info("Campus Tour Simulation Node initialized")

    def run_simulation(self):
        self.simulate_visitor_1()
        self.simulate_visitor_2_and_3()
        self.simulate_visitor_4()
        self.simulate_visitor_5()
        
        self.campus_map.visualize_scenario()
        self.get_logger().info("Campus Tour Simulation Completed")

    def simulate_visitor_1(self):
        self.update_agent_position('CI Agent', 'Entrance')
        self.update_agent_position('Visitor 1', 'Entrance')
        self.campus_map.add_interaction('CI Agent', 'Visitor 1', 'Entrance', 'Meet')
        
        self.navigate('CI Agent', 'Visitor 1', 'CSE Dept')
        self.campus_map.add_interaction('CI Agent', 'BI CSE', 'CSE Dept', 'Handover')
        
        auth_result = self.request_authorization('Visitor 1', 'CSE Dept')
        if auth_result:
            self.campus_map.add_interaction('BI CSE', 'Visitor 1', 'CSE Dept', 'Authorize')
        else:
            self.get_logger().warn("Visitor 1 not authorized for CSE Dept")

    def simulate_visitor_2_and_3(self):
        self.update_agent_position('CI Agent', 'Entrance')
        self.update_agent_position('Visitor 2', 'Entrance')
        self.campus_map.add_interaction('CI Agent', 'Visitor 2', 'Entrance', 'Meet')
        
        self.navigate('CI Agent', 'Visitor 2', 'AI Dept')
        self.campus_map.add_interaction('CI Agent', 'BI AI', 'AI Dept', 'Handover')
        
        auth_result = self.request_authorization('Visitor 2', 'AI Dept')
        if auth_result:
            self.campus_map.add_interaction('BI AI', 'Visitor 2', 'AI Dept', 'Authorize')
            
            # Simulate 10-minute meeting
            self.get_logger().info("Visitor 2 meeting for 10 minutes")
            
            # Visitor 3 arrives
            self.update_agent_position('Visitor 3', 'Entrance')
            self.campus_map.add_interaction('CI Agent', 'Visitor 3', 'Entrance', 'Meet')
            self.navigate('CI Agent', 'Visitor 3', 'AI Dept')
            self.campus_map.add_interaction('BI AI', 'Visitor 3', 'AI Dept', 'Queue')
            
            # After 10 minutes
            self.campus_map.add_interaction('BI AI', 'Visitor 2', 'AI Dept', 'End Meeting')
            self.campus_map.add_interaction('BI AI', 'Visitor 3', 'AI Dept', 'Start Meeting')
        else:
            self.get_logger().warn("Visitor 2 not authorized for AI Dept")

    def simulate_visitor_4(self):
        self.update_agent_position('CI Agent', 'Entrance')
        self.update_agent_position('Visitor 4', 'Entrance')
        self.campus_map.add_interaction('CI Agent', 'Visitor 4', 'Entrance', 'Meet')
        
        self.navigate('CI Agent', 'Visitor 4', 'Cafeteria')
        self.campus_map.add_interaction('CI Agent', 'BI Cafeteria', 'Cafeteria', 'Handover')
        
        oos_result = self.request_out_of_service('BI Cafeteria', 10)  # 10 minutes OOS
        if oos_result:
            self.campus_map.add_interaction('BI Cafeteria', 'Visitor 4', 'Cafeteria', 'Host (OOS)')
            # Simulate 10-minute hosting
            self.get_logger().info("BI Cafeteria hosting Visitor 4 for 10 minutes")
        else:
            self.get_logger().warn("BI Cafeteria unable to go OOS")

    def simulate_visitor_5(self):
        self.update_agent_position('CI Agent', 'Entrance')
        self.update_agent_position('Visitor 5', 'Entrance')
        self.campus_map.add_interaction('CI Agent', 'Visitor 5', 'Entrance', 'Meet')
        
        self.navigate('CI Agent', 'Visitor 5', "Director's Office")
        self.campus_map.add_interaction('CI Agent', 'BI Director', "Director's Office", 'Handover')
        
        auth_result = self.request_authorization('Visitor 5', "Director's Office")
        if auth_result:
            self.campus_map.add_interaction('BI Director', 'Visitor 5', "Director's Office", 'Authorize')
        else:
            self.get_logger().warn("Visitor 5 not authorized for Director's Office")
            self.campus_map.add_interaction('BI Director', 'Visitor 5', "Director's Office", 'Deny Access')

    def update_agent_position(self, agent_name, location):
        msg = AgentPosition()
        msg.agent_name = agent_name
        msg.x = 0.0  # Set the x position, you can modify this according to your logic
        msg.y = 0.0  # Set the y position, you can modify this according to your logic
        self.agent_position_pub.publish(msg)
        self.campus_map.update_agent_position(agent_name, location)

    def navigate(self, agent_id, visitor_id, destination):
        msg = NavReq()
        msg.visitor_id = visitor_id
        msg.building = destination  # Updated from destination to building
        self.nav_req_pub.publish(msg)

    def nav_response_callback(self, msg):
        self.get_logger().info(f"Navigation response: Agent {msg.visitor_id} arrived at {msg.building}")
        self.update_agent_position(msg.visitor_id, msg.building)

    def visitor_request_callback(self, msg):
        self.get_logger().info(f"Visitor request: {msg.agent_name} requesting access at ({msg.x}, {msg.y})")

    def request_authorization(self, visitor_id, location):
        req = Auth.Request()
        req.visitor_id = visitor_id  # Use visitor_id as per the service definition
        
        future = self.auth_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            return future.result().authorized
        else:
            self.get_logger().error('Service call failed')
            return False

    def request_out_of_service(self, bi_agent, duration):
        req = OOS.Request()
        req.bi_agent = bi_agent
        req.duration = duration
        
        future = self.oos_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            return future.result().acknowledged  # Corrected to acknowledged
        else:
            self.get_logger().error('Service call failed')
            return False

def main(args=None):
    rclpy.init(args=args)
    node = CampusTourSimulation()
    try:
        node.run_simulation()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
