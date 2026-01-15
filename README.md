# WATonomous ASD Admissions Assignment

A ROS2 assignment utilizing GazeboSim and FoxGlove to simulate a robot autonomously navigating through terrain, and avoiding obstacles using the A* pathfinding algorithm. Used Pure Pursuit to follow planned trajectories. 

Used for admission to Waterloo WATonomous Design Team

Programmed these 4 ROS2 Nodes: 

1. Costmap Node: Creates local map of surroundings using simulated LIDAR)
2. Map Memory Node: Aggregrates into a global map, prioritizing new data and automatically cleaning up false positives
3. Planner Node: Uses A* algorithm to find the best possible path to destination, avoiding obstacles detected in Map Memory
4. Control Node: Calculates needed linear and angular velocity for the robot to follow this trajectory accurately, using Pure Pursuit Controller

https://github.com/user-attachments/assets/d94596e3-8b4f-477c-95b9-01bb7476547b

## Prerequisite Installation
These steps are to setup the monorepo to work on your own PC. We utilize docker to enable ease of reproducibility and deployability.

> Why docker? It's so that you don't need to download any coding libraries on your bare metal pc, saving headache :3

1. This assignment is supported on Linux Ubuntu >= 22.04, Windows (WSL), and MacOS. This is standard practice that roboticists can't get around. To setup, you can either setup an [Ubuntu Virtual Machine](https://ubuntu.com/tutorials/how-to-run-ubuntu-desktop-on-a-virtual-machine-using-virtualbox#1-overview), setting up [WSL](https://learn.microsoft.com/en-us/windows/wsl/install), or setting up your computer to [dual boot](https://opensource.com/article/18/5/dual-boot-linux). You can find online resources for all three approaches.
2. Once inside Linux, [Download Docker Engine using the `apt` repository](https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository)
3. You're all set! You can begin the assignment by visiting the WATonomous Wiki.

Link to Onboarding Assignment: https://wiki.watonomous.ca/
