import rclpy
from crewai import Crew, Task as CrewTask  # Renamed to avoid conflicts
from rclpy.node import Node
from agents.CI_agents import CampusInchargeAgent
from agents.BI_agents import BuildingInchargeAgent
from agents.Visitor_agents import VisitorAgent
from utils.campus_map import CampusMap
from scripts.task import Task
import yaml

def load_config(file_path):
    with open(file_path, 'r') as file:
        return yaml.safe_load(file)

def run_simulation(node):
    # Load configurations for agents
    ci_config = load_config('src/campus_tour/campus_tour/config/CI_agents.yaml')
    bi_config = load_config('src/campus_tour/campus_tour/config/BI_agents.yaml')
    visitor_config = load_config('src/campus_tour/campus_tour/config/Visitor_agents.yaml')

    # Initialize agents
    ci_agent = CampusInchargeAgent(ci_config, node)
    bi_agents = [
        BuildingInchargeAgent({**bi_config, 'name': 'BI CSE', 'building': 'CSE Dept'}, node),
        BuildingInchargeAgent({**bi_config, 'name': 'BI AI', 'building': 'AI Dept'}, node),
        BuildingInchargeAgent({**bi_config, 'name': 'BI Cafeteria', 'building': 'Cafeteria'}, node),
        BuildingInchargeAgent({**bi_config, 'name': 'BI Director', 'building': "Director Office"}, node)
    ]

    visitors = [
        VisitorAgent({**visitor_config, 'name': 'Visitor 1', 'host': 'Host in CSE Dept'}, node),
        VisitorAgent({**visitor_config, 'name': 'Visitor 2', 'host': 'Host in AI Dept'}, node),
        VisitorAgent({**visitor_config, 'name': 'Visitor 3', 'host': 'Host in AI Dept'}, node),
        VisitorAgent({**visitor_config, 'name': 'Visitor 4', 'host': 'BI Cafeteria'}, node),
        VisitorAgent({**visitor_config, 'name': 'Visitor 5', 'host': "Host in Director Office"}, node)
    ]

    # Initialize campus map
    campus_map = CampusMap()

    # Define tasks
    tasks = [
        CrewTask(description="Process Visitor 1 to CSE Dept", agent=ci_agent, type="VisitorTask", expected_output="No output expected"),
        CrewTask(description="Process Visitor 2 to AI Dept", agent=ci_agent, type="VisitorTask", expected_output="No output expected"),
        CrewTask(description="Process Visitor 3 to AI Dept", agent=ci_agent, type="VisitorTask", expected_output="Queued - Cannot meet host till 10 mins"),
        CrewTask(description="Process Visitor 4 to Cafeteria", agent=ci_agent, type="VisitorTask", expected_output="No output expected"),
        CrewTask(description="Process Visitor 5 to Director Office", agent=ci_agent, type="VisitorTask", expected_output="No output expected")
    ]

    # Initialize the crew
    campus_tour_crew = Crew(
        agents=[ci_agent] + bi_agents + visitors,
        tasks=tasks
    )

    node.get_logger().info("Starting Campus Tour Simulation")

    # Execute tasks and track paths
    for task in tasks:
        node.get_logger().info(f"Executing task: {task.description}")
        node.get_logger().info(f"Task attributes: {vars(task)}")
        
        visitor_name = task.description.split()[2]
        destination = ' '.join(task.description.split()[4:])
        # path = campus_map.get_path(CampusMap.ENTRANCE, destination)
        # campus_map.add_visitor_path(visitor_name, path)

        # if "Visitor 1" in task.description:
        #     path = campus_map.get_path(CampusMap.ENTRANCE, "CSE Dept")
        #     campus_map.add_visitor_path('Visitor 1', path)
        # elif "Visitor 2" in task.description:
        #     path = campus_map.get_path(CampusMap.ENTRANCE, "AI Dept")
        #     campus_map.add_visitor_path('Visitor 2', path)
        # elif "Visitor 3" in task.description:
        #     path = campus_map.get_path("AI Dept", "AI Dept")  # Assuming Visitor 3 is already in AI Dept
        #     campus_map.add_visitor_path('Visitor 3', path)
        # elif "Visitor 4" in task.description:
        #     path = campus_map.get_path(CampusMap.ENTRANCE, "Cafeteria")
        #     campus_map.add_visitor_path('Visitor 4', path)
        # elif "Visitor 5" in task.description:
        #     path = campus_map.get_path(CampusMap.ENTRANCE, "Director Office")
        #     campus_map.add_visitor_path('Visitor 5', path)

        # try:
        #     task.execute()  # Ensure the method exists
        # except AttributeError as e:
        #     node.get_logger().error(f"Error executing task: {e}")
        #     continue  # Skip this task and move to the next

        # try:
        #     # Use the Crew to execute the task
        #     result = campus_tour_crew.execute_task(task)
        #     node.get_logger().info(f"Task result: {result}")
        # except Exception as e:
        #     node.get_logger().error(f"Error executing task: {e}")
        #     continue
        try:
            path = campus_map.get_path(CampusMap.ENTRANCE, destination)
            campus_map.add_visitor_path(visitor_name, path)
            node.get_logger().info(f"Path for {visitor_name}: {' -> '.join(path)}")
        except ValueError as e:
            node.get_logger().error(f"Error finding path: {e}")
            continue

        try:
            # result = campus_tour_crew.execute_task(task)
            node.get_logger().info(f"Simulating execution of task: {task.description}")
        except Exception as e:
            node.get_logger().error(f"Error executing task: {e}")
            continue

    # campus_map.animate_paths()  
    node.get_logger().info("Campus Tour Simulation Completed")

def main(args=None):
    rclpy.init(args=args)  
    node = Node('camp_sim_launch')  
    try:
        run_simulation(node) 
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node() 
        rclpy.shutdown()  

if __name__ == '__main__':
    main()
