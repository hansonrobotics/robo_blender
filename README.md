# Robo Blender

ROS node which starts within the blender and have direct access to blender context. Uses different modes to control rig.
Different modes can be used for different rigs (currently used with basic neck rig, and also with animations rig). 
Currently two rigs are supported:
  * Neck rig (included).
  * Animations Rig by Beorn

Listens for /cmd_blender to switch between the different blender nodes

## Modes
Main modes currently in use:
### Animations
Plays already predefined animations

##### Topics subscribed:
  * cmd_animations(std_msgs/String) - colon separated string which sends command (play,stop, pause) and optionaly the animation name.

##### Outputs:
  * full_head - publishes expression neck and eyes information.

### LookAround
Moving head and the eyes. Enabled once robot seaks attention.

##### outputs
  * neck_euler - publishes neck angle
  * eyes - publishes eyes movements

### Manual Head
Used for development purpuses. Allows designer to change rig, while seeing the actuall output on the robot.

##### Outputs:
  * full_head  - publishes expression neck and eyes information.

### TrackDev
Current head tracking topic. Has primary and secondary target  that can be changed during runtime.

##### Topics subscribed:
  * /tracking_action (eva_behavior/tracking_action) - Listens for information on objects to track.
#### Inputs
  * pi_vision (RegionOfInterest) - topic publishing ROI  for tracking
  * glancetarget (RegionOfInterest) - topic publishing ROI for glancing
##### outputs
  * neck_euler - publishes neck angle
  * eyes - publishes eyes movements

### Dummy
Idle.

## Inputs
Mainly one input class is currently used:
  * RegionOfInterest - Listens to specified topic for sensor_msgs/RegionOfInterest message. Converts it to 3D space in blender and updates the blender object position in the blender.
  * Faceshift - allows input from faceshift, changes shapekeys of the face mesh.

## Outputs
Outputs are used to publish rig data back to ROS. Ros topics are defined in config.

The following outputs are currently used:
  * neck_euler - gets neck's Euler rotation, publishes as basic_head_api.msg/PointHead
  * face - gets mesh shapekeys and publishes them as pau2motors/pau. 
  * eyes - gets eyes angles and publishes them as pau2motors/pau
  * eyes_beorn - gets Beorn's rig eyes angles and publishes in pau2motors/pau message on specified topic.
  * neck_euler_beorn  - gets Beorn's rig neck rotation in Euler angles and publishes in basic_head_api.msg/PointHead
  * full_head - combines face, neck_euler_beorn, eyes_beorn outputs to one.












