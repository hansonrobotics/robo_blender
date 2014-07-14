Robo Blender
============

This Blender rig uses **inputs** package to map sensor information into Blender space and **outputs** package to map the Blender rig onto ROS controllers.

The control modes (the way input data is used to control the Blender rig) can be activated by sending the mode name (which is equivalent to the name of a module in folder **modes**) to `cmd_blendermode`.

See **inputs/__init__.py**, **outputs/__init__.py** and **modes/__init__.py** for more documentation.

As of 14 July the system supports the neck, eyes and face outputs, camera tracking inputs and the faceshift input (which serves as a general head pau message input for the time being).
