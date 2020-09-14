# ROS Motion Builder

The ROS Motion Builder project is an open-source tool that allows the design and creation of predefined motions for a robot, as well as an RQT-based GUI, and a Javascript library to interface with said node.

## Packages
This project is composed of 4 packages.

### ros_motion_builder_msgs
This package contains message, action and service definitions for the node's communication.

### ros_motion_builder
This is the main package of the project, containing both the data model and the main node for the project.

### rqt_motion_builder
This package contains an RQT-based frontend for the project, which allows simple control of the motion creation pipeline.

### ros_motion_builder_js
This folder contains a simple Javascript library, based on roslibjs, to simplify the interaction with the motion creation pipeline. In addition this folder contains a ready-to-use html front-end for the project.