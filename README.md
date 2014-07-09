Robo Blender
============

This Blender rig uses **inputs.py** to map sensor information into Blender space and **outputs.py** to map the Blender rig onto ROS controllers.

The control modes (the way input data is used to control the Blender rig) can be activated by sending the mode name (which is equivalent to the name of a module in folder **modes**) to `cmd_blendermode`.

See **inputs.py**, **outputs.py** and **modes/__init__.py** for more documentation.

As of 9 July the system supports only the neck output and camera tracking inputs.
