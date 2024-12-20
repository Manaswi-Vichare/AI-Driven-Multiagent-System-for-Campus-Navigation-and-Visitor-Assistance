#!/usr/bin/env python3
import rclpy
from utils.visual import visualize_campus

def main():
    rclpy.init()  # Initialize the ROS 2 context
    try:
        visualize_campus()
    except KeyboardInterrupt:  # Catch keyboard interrupt to gracefully shut down
        pass
    finally:
        rclpy.shutdown()  # Ensure proper shutdown

if __name__ == '__main__':
    main()
