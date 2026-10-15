---
layout: single
title: "Article Revision and Conference Acceptance"
date: 2026-10-15
excerpt: "Addressing the reviewers' feedback, validating the robot's navigation capabilities, and working on object carrying."
categories: blog
---

## Addressing the Reviewers' Feedback

After submitting our article to the conference, we received a mixed review. While one reviewer was positive about our project, the other raised concerns about some features that had not yet been experimentally validated. They also questioned the project's originality, pointing out similarities with the [rl_sar repository](https://github.com/fan-ziqi/rl_sar).

The conference organizers initially asked us to submit a revised version of the article addressing these concerns.

### Validating the Robot's Navigation Capabilities

One of the reviewer's main concerns was that some of the capabilities described in our article had not been sufficiently tested. To address this issue, I created a new simulation world with markers placed on the floor and made the robot follow a route connecting them.

This experiment was intended to demonstrate that our locomotion system could be integrated with a navigation system, allowing the robot to follow a predefined route rather than simply walking in place or moving in a specific direction.

### How Does Our Project Differ from rl_sar?

Addressing the concerns about the project's originality was more challenging.

The [rl_sar repository](https://github.com/fan-ziqi/rl_sar) contains a ROS 2 node that uses neural networks to control a robot's locomotion, which is also one of the main goals of our project. Although the two projects share some similarities, they differ in their software environments and intended scope.

For example, rl_sar targets ROS 2 Foxy, whereas our project uses ROS 2 Jazzy. However, we considered this difference insufficient on its own to clearly distinguish the two projects. Therefore, we focused on the additional capabilities we plan to develop.

While rl_sar focuses on locomotion, our long-term goal is to develop a more versatile humanoid robot capable of performing three main tasks:

- **Whole-body locomotion:** using the entire body to maintain balance and improve stability while walking.
- **Object carrying:** walking while keeping the arms available to carry objects, such as a box.
- **Fall recovery:** getting back up autonomously after falling.

These capabilities represent different challenges in humanoid robotics and are intended to extend the functionality of our locomotion system.

### Conference Acceptance

While I was working on the object-carrying functionality, we received another email from the conference organizers. A third reviewer had evaluated our article and given it a positive assessment, so the conference accepted our submission without requiring a revised version.

Nevertheless, we considered the previous reviewer's concerns valid and decided to continue improving the project. We wanted our claims to be supported by experiments and to make the project's contributions clearer, regardless of whether a revised submission was still required.

## Working on Object Carrying

Making the robot walk while carrying an object turned out to be more challenging than expected.

To simulate carrying a box, I initially trained the neural network with the robot's arms held in a fixed position. However, the resulting network did not work correctly in Gazebo.

After checking the ROS 2 nodes for potential errors, I decided to train the network again, this time without providing the arm joint positions and velocities as inputs.

This new network worked in Gazebo, suggesting that the arm-related inputs had been contributing to the problem. During training in MuJoCo, fixing the arms appeared to produce joint-state values that were not suitable for direct use by the network in the Gazebo simulation.

Removing these inputs allowed us to obtain a network that could run in Gazebo. However, this did not necessarily mean that the robot had learned to carry objects reliably.

### Results and Next Steps

Although we managed to run the new network in Gazebo, the results were not satisfactory enough to consider object carrying a completed feature.

We therefore decided not to present it as an implemented capability of the project. Instead, we will continue working on it as part of the project's future development.

This experiment highlighted an important challenge when training locomotion policies in simulation: the network's inputs must remain consistent with the robot's state representation across different simulation environments.

## Summary

- **Conference acceptance:** Our article was accepted without requiring a revised submission.
- **Navigation demonstration:** I created a simulation environment in which the robot follows a predefined route marked on the floor.
- **Object carrying:** I trained and tested a new locomotion network in Gazebo, although its performance was not yet sufficient for reliable object carrying.
- **Future work:** We will continue developing the robot's ability to carry objects while improving the validation of our system's capabilities.