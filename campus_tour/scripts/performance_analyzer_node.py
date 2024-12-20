#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from utils.perf_analyzer import PerformanceAnalyzer

def main(args=None):
    rclpy.init(args=args)  # Initialize rclpy
    analyzer_node = PerformanceAnalyzer()  # Create the analyzer node
    
    try:
        rclpy.spin(analyzer_node)  # Keep the node running
    except KeyboardInterrupt:
        pass
    finally:
        analyzer_node.destroy_node()  # Cleanup
        rclpy.shutdown()  # Shutdown rclpy

if __name__ == '__main__':
    main()
