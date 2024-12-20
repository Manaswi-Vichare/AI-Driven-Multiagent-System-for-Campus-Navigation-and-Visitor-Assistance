import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from campus_tour_interfaces.msg import PerformanceMetrics

class PerformanceAnalyzer(Node):
    def __init__(self):
        super().__init__('performance_analyzer_node')  # Initialize the node with a name
        self.ci_metrics = {}
        self.bi_metrics = {}
        
        # Subscribe to the '/agent_performance' topic
        self.subscription = self.create_subscription(
            PerformanceMetrics,
            '/agent_performance',
            self.update_metrics,
            10  # QoS History depth
        )

    def update_metrics(self, msg):
        if msg.agent_type == 'CI':
            self.ci_metrics[msg.agent_name] = {
                'visitors_entertained': msg.visitors_entertained,
                'violations': msg.violations
            }
        elif msg.agent_type == 'BI':
            self.bi_metrics[msg.agent_name] = {
                'ci_agents_guided': msg.ci_agents_guided,
                'violations': msg.violations
            }

    def analyze_performance(self):
        self.get_logger().info("Performance Analysis:")
        for agent, metrics in self.ci_metrics.items():
            self.get_logger().info(f"{agent}: Visitors entertained: {metrics['visitors_entertained']}, Violations: {metrics['violations']}")
        for agent, metrics in self.bi_metrics.items():
            self.get_logger().info(f"{agent}: CI agents guided: {metrics['ci_agents_guided']}, Violations: {metrics['violations']}")

def main(args=None):
    rclpy.init(args=args)
    analyzer = PerformanceAnalyzer()
    rate = analyzer.create_rate(1)  # 1 Hz rate

    try:
        while rclpy.ok():
            rclpy.spin_once(analyzer)
            analyzer.analyze_performance()
            rate.sleep()
    except KeyboardInterrupt:
        pass
    finally:
        analyzer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
