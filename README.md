# AI-Driven-Multiagent-System-for-Campus-Navigation-and-Visitor-Assistance
**Objectives:** 
1. Designed and implemented a multiagent system using CrewAI framework and ROS to facilitate campus navigation for visitors.
2. Developed inter-agent communication protocols for coordination between Campus Incharge (CI) Agents, Building Incharge (BI) Agents, and Visitor Agents, ensuring smooth navigation and visitor management.
3. Simulated system interactions with varying visitor arrivals, hosts, and meeting durations, visualizing navigation paths as graphs and logging inter-agent communication.
4. Implemented penalty-based performance metrics to evaluate agents’ efficiency, including OOS (Out of Service) violations and adherence to committed durations.

**Commands:** \
To build the ROS2 environment:
```
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```
To create the main implementation code folder:
```
ros2 pkg create --build-type ament_python campus_tour
```
To create the communication code folder:
```
ros2 pkg create --build-type ament_python campus_tour_interfaces
```
Initiate the ROS environment:
```
cd ~/ros2_ws
colcon build
source ~/ros2_ws/install/setup.bash
```
Run the agent nodes and performance analyzer:
```
ros2 run campus_tour ci_agent_node
ros2 run campus_tour bi_agent_node
ros2 run campus_tour visitor_node
ros2 run campus_tour performance_analyzer_node
ros2 run campus_tour visualization_node
```
Run the virtual simulation:
```
ros2 launch campus_tour camp_sim_launch.py
#or
ros2 run campus_tour campus_map.py
```

**Example Simulation:**
1. Visitor 1 is escorted to the CSE Department, where the BI Agent provides navigation assistance after verifying their authorization.
2. Visitor 2 is escorted to the AI Department, and the BI Agent assists with navigation after verifying their authorization.
3. Visitor 3 is escorted to the Cafeteria, where the BI Agent goes OOS for 2 minutes to host after verifying their authorization.
4. Visitor 4 waits for 2 minutes while Visitor 2 meets the host, then is escorted to the AI Department after Visitor 2 leaves.
5. Visitor 5 is denied access to the Director's Office by the BI Agent due to lack of proper authorization.

<img src="https://github.com/user-attachments/assets/474b7aaf-4079-41df-abca-b2314aee79f0" width="600" />
