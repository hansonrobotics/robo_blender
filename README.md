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

Currently only one rig is included:
  * robo.blend

The last working version of the Einstein rig is in the branch named
"einstein-dev".

## Pre-requisites
The following packages need to be installed:

    apt-get install python3-yaml python3-rospkg

Caution: python3+ros has issues; see this bug:
https://github.com/ros-infrastructure/rospkg/issues/71

This can be worked-around as follows:

    git clone git://github.com/ros/rospkg.git
    cd rospkg
    python3 setup.py install
    cd ..
    git clone git://github.com/ros-infrastructure/catkin_pkg.git
    cd catkin_pkg
    python3 setup.py install


## Running
To run, start blender, and load the robo.blend rig file.  The ROS node
is not started until one of the following is performed:

 * Hover mouse over the text editor and press Alt+P
 * Select the text-editor menu "Edit->Run Script" item.
   (the text-editor menu is at bottom-left).
 * Start blender as `blender robo.blend -P startup.py`

Verify that the ROS node has started:  `rostopic list -v` should show
`/cmd_blendermode` as a subscribed topic.

## Modes

Listens for /cmd_blender to switch between the different blender nodes.
The currently supported modes are:
 * LookAround  -- random looking about
 * Manual Head -- manual control from blender, debugging
 * TrackDev    -- head/eyes track a region of interest
 * Animations  -- menu of scripted animations
 * Dummy       --

### LookAround
Move the head and eyes in an endless cycle. Movements are defined via
a python script in the rig.  The movements are randomly generated, with
eyes moving faster than head.

For example:
   rostopic pub /cmd_blendermode std_msgs/String LookAround

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
Eyes and head track targets defined by region of interest (typically
obtained via camera viedo processing).  Maintains two targets from
the scene. The target to be tracked is selected with the
/tracking_action topic.

For example:
   rostopic pub /cmd_blendermode std_msgs/String TrackDev

##### Topics subscribed:
  * /tracking_action (eva_behavior/tracking_action) - Listens for
    information on objects to track.


#### Inputs
  * chest_pivision (NRegionsOfInterest) - populates the scene with ROI
    from chest camera
  * eye_pivision (NRegionsOfInterest) - populates the scene with ROI
    from eye camera

##### Outputs
  * neck_euler - publishes neck angle
  * eyes - publishes eyes movements


### Animations
Plays pre-defined animations.  These are defined as object actions
within the blender rig.  The list of valid animations can be obtained
with the /animations_list topic; this topic list is NOT published until
the rig is placed in animation mode.

No error is thrown if an invalid animation name is specified.

Animations are currently implemented only the Beorn rig.

For example:
   rostopic pub /cmd_blendermode std_msgs/String Animations

##### Topics subscribed:
  * cmd_animations(std_msgs/String) - colon separated string which
    sends command (play,stop, pause) follwed optionaly the animation name.

    For example:
    rostopic pub /cmd_blendermode std_msgs/String play:blah

##### Outputs:
  * full_head - publishes expression neck and eyes information.


### Dummy
Idle.

For example:
   rostopic pub /cmd_blendermode std_msgs/String Dummy

## Inputs
Mainly one input class is currently used:
  * NRegionsOfInterest - Listens to the specified *eventtopic* for
    eva_behavior/tracking_event messages, which inform about new and
    removed sensor_msgs/RegionOfInterest topics of the format specified
    in *sourcetopic*. Converts them to 3D space in blender and updates
    their position.
  * RegionOfInterest - Listens to specified topic for
    sensor_msgs/RegionOfInterest message. Converts it to 3D space
    in blender and updates the blender object position in the blender.
  * Faceshift - allows input from faceshift, changes shapekeys
    of the face mesh.

## Outputs
Outputs are used to publish rig data back to ROS.  ROS topics are
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
