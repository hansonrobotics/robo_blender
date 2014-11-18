# Robo Blender

This provides a ROS node which may be used to control Blender rigs,
and to publish the resulting rig position information (for the neck,
face, eyes).  The position information can then be used to drive motors
(for example).  The https://github.com/hansonrobotics/pau2motors ROS
node will listen to the published messages to control the Einstein
and Dmitry robot heads.

The node is started within Blender, and has direct access to the
Blender context.  It uses different modes to control the rig, with
different modes used for different rigs. 

Currently two rigs are supported:
  * Neck rig (included).
  * Animations Rig by Beorn

Listens for /cmd_blender to switch between the different blender nodes.

## Modes
The currently supported modes are:

### Animations
Plays animations defined within the rig itself (i.e. defined as object
actions).  The neck rig dfines the animations blah, blah and blah. 
Beorn's rig defines animations blah and blah.  An invalid animation name
will XXX (? throw an error? be silently ignored?)

##### Topics subscribed:
  * cmd_animations(std_msgs/String) - colon separated string which
    sends command (play,stop, pause) and optionaly the animation name.

##### Outputs:
  * full_head - publishes expression neck and eyes information.

### LookAround
Move the head and eyes. Enabled once robot seeks attention. Movements
are defined via a python script in the rig.  The movements are randomly
generated, with eyes moving faster than head.

##### Inputs:
  * (none)

##### Outputs
  * neck_euler - publishes neck angle
  * eyes - publishes eyes movements

### Manual Head
Used for animation development and debugging. Allows the designer
to control and move the rig, thus controlling the physical robot head.

##### Outputs:
  * full_head  - publishes expressions for neck and eyes positions.

### TrackDev
Current head tracking topic. Has primary and secondary target that
can be changed during runtime.

##### Topics subscribed:
  * /tracking_action (eva_behavior/tracking_action) - Listens for
    information on objects to track.

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
  * RegionOfInterest - Listens to specified topic for 
    sensor_msgs/RegionOfInterest message. Converts it to 3D space
    in blender and updates the blender object position in the blender.
  * Faceshift - allows input from faceshift, changes shapekeys 
    of the face mesh.

## Outputs
Outputs are used to publish rig data back to ROS.  Ros topics are 
defined in config.

The following outputs are currently used:
  * neck_euler - gets neck's Euler rotation, publishes as
    basic_head_api.msg/PointHead
  * face - gets mesh shapekeys and publishes them as pau2motors/pau. 
  * eyes - gets eyes angles and publishes them as pau2motors/pau
  * eyes_beorn - gets Beorn's rig eyes angles and publishes in 
    pau2motors/pau message on specified topic.
  * neck_euler_beorn  - gets Beorn's rig neck rotation in Euler angles
    and publishes in basic_head_api.msg/PointHead
  * full_head - combines face, neck_euler_beorn, eyes_beorn outputs to one.

