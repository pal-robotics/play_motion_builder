# Play Motion Builder

The Play Motion Builder project is an open-source tool that allows the design and creation of predefined motions for a robot, as well as an RQT-based GUI, and a Javascript library to interface with said node.

## Packages
This project is composed of 4 packages.

### play_motion_builder_msgs
This package contains message, action and service definitions for the node's communication.

### play_motion_builder
This is the main package of the project, containing both the data model and the main node for the project.

### rqt_play_motion_builder
This package contains an RQT-based frontend for the project, which allows simple control of the motion creation pipeline.

### play_motion_builder_js
This folder contains a simple Javascript library, based on roslibjs, to simplify the interaction with the motion creation pipeline. In addition this folder contains a ready-to-use html front-end for the project.

## Acknowledgements
***
<!-- 
    ROSIN acknowledgement from the ROSIN press kit
    @ https://github.com/rosin-project/press_kit
-->

<a href="http://rosin-project.eu">
  <img src="http://rosin-project.eu/wp-content/uploads/rosin_ack_logo_wide.png" 
       alt="rosin_logo" height="60" >
</a>

Supported by ROSIN - ROS-Industrial Quality-Assured Robot Software Components.  
More information: <a href="http://rosin-project.eu">rosin-project.eu</a>

<img src="http://rosin-project.eu/wp-content/uploads/rosin_eu_flag.jpg" 
     alt="eu_flag" height="45" align="left" >  

This project has received funding from the European Union’s Horizon 2020  
research and innovation programme under grant agreement no. 732287. 
