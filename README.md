# Forward-and-Inverse-Kinematics-of-the-KUKA-KR6-R900
Forward and Inverse Kinematics implementation for the KUKA KR 6 R900 sixx 6-DOF robot manipulator using ROS 2 Jazzy and RViz2.

## Project Overview
This repository contains the complete kinematic modeling and ROS 2 implementation for the **KUKA KR 6 R900 sixx** 6-DOF industrial manipulator, developed for the **IMT-342 Robotics** course at Universidad Católica Boliviana "San Pablo".

The project covers analytical and numerical derivations of Denavit–Hartenberg (DH) parameters, Forward Kinematics (FK), Geometric Jacobian matrix, and Inverse Kinematics (IK), with simulation and numerical validation in ROS 2 Jazzy and RViz2.

## Key Features
* **Kinematic Modeling:** Standard Denavit–Hartenberg (DH) frame assignment, individual transformation matrices, and end-effector pose formulation.
* **Jacobian & Singularities:** Geometric Jacobian formulation and singularity analysis for velocity kinematics.
* **Iterative Inverse Kinematics:** Numerical solver utilizing Pseudoinverse / Damped Least Squares (DLS) with joint limits enforcement and convergence checks.
* **ROS 2 Integration:** Custom Python nodes (`fk_node` and `ik_node`) running under ROS 2 Jazzy with CycloneDDS middleware.
* **Visualization:** Joint state control and real-time visualization using `robot_state_publisher`, `joint_state_publisher_gui`, and RViz2.

## Prerequisites
* **Operating System:** Ubuntu 24.04 LTS
* **ROS 2 Version:** Jazzy Jalisco (`rmw_cyclonedds_cpp`)
* **Python Libraries:** `numpy`, `scipy`, `sympy`
