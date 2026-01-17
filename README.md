# WATonomous ASD Admissions Assignment

A ROS2 assignment utilizing GazeboSim and FoxGlove to simulate a robot autonomously navigating through terrain, and avoiding obstacles using the A* pathfinding algorithm. Used Pure Pursuit to follow planned trajectories. 

Used for admission to Waterloo WATonomous Design Team

Programmed these 4 ROS2 Nodes: 

  1. **Costmap Node:** Creates local map of surroundings using simulated LIDAR
  2. **Map Memory Node:** Aggregrates into a global map, prioritizing new data and automatically cleaning up false positives
  3. **Planner Node:** Uses **A*** algorithm to find the best possible path to destination, avoiding obstacles detected in Map Memory
  4. **Control Node:** Calculates needed linear and angular velocity for the robot to follow this trajectory accurately, using **Pure Pursuit Controller**

https://github.com/user-attachments/assets/d94596e3-8b4f-477c-95b9-01bb7476547b
