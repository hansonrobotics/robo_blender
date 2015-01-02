# Robo Blender

This provides a ROS node which may be used to control Blender rigs,
and to publish the resulting rig position information (for the neck,
face, eyes).  The position information can then be used to drive motors
(for example).  The 
[pau2motors](http://github.com/hansonrobotics/pau2motors) ROS node
will listen to the published messages to control the Einstein and
Arthur robot heads.

The node is started within Blender, and has direct access to the
Blender context.  It uses different modes to control the rig, with
different modes used for different rigs.

Currently two rigs are included:
  * robo.blend -- Arthur rig, can look around, track objects.
  * animate-test.blend -- Arthur rig, can animate four basic expressions.

Although both of these rigs are "Arthur", they are not compatible
with each other.

The last working version of the Einstein rig is in the branch named
"einstein-dev". It has not been kept up to date.

# Running
The easiest way to run this node is to use it with a Docker container.
Docker containers specify all of the pre-requisite software components 
that are needed to in order for a demo to be run.  An all-in-one
containder for this node has been defined at
[opencog/ros](http://github.com/opencog/ros), in the ros-indigo-animation
image.  The demo process is then:

 * Install docker
 * git clone http://github.com/opencog/ros
 * cd opencog/ros/indigo
 * ./build-all.sh
 * cd animation and follow instructions in the README file there.

If you do the above, you can skip the Manual Install steps below.

## Manual Install and Operation
### Pre-requisites
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

### Co-requisites
The tracking mode requires that 
[eva_behavior](http://github.com/hansonrobotics/eva_behavior) ROS node
be set up and running.  Additional instructions in the 'cookbook'
section, below.

### Manual start
To run, start blender, and load either the robo.blend or the
animate-test.blend rig file.  The ROS node is not started until
one of the following is performed:

 * Hover mouse over the text editor and press Alt+P
 * Select the text-editor menu "Edit->Run Script" item.
   (the text-editor menu is at bottom-left).
 * Start blender as `blender robo.blend --enable-autoexec -P startup.py`

Verify that the ROS node has started:  `rostopic list -v` should show
`/cmd_blendermode` as a subscribed topic.

Don't forget to start `roscore` first.

# Modes

The rigs have several different modes in which they can operate. These
are:

 * LookAround  -- Random looking about.
 * TrackDev    -- Head/eyes track a region of interest.
 * Animations  -- Menu of scripted animations.
 * Manual Head -- Manual control from blender, debugging
 * Dummy       -- Do nothing. Neutral expression.

The mode is set by sending one of the above modes, as a string,
to the `/cmd_blender` topic.  For example:

`rostopic pub --once /cmd_blendermode std_msgs/String LookAround`

Both rigs listen for `/cmd_blender` to switch between the different
blender nodes.

Currently, only the `robo.blend` rig can respond to the LookAround
and TrackDev modes.  In contrast, only the `animate.blend` rig can
respond to the Animations mode.  Sorry!

### LookAround
Move the head and eyes in an endless cycle. Movements are defined via
a python script in the rig.  The movements are randomly generated, with
eyes moving faster than head.

For example:
   `rostopic pub --once /cmd_blendermode std_msgs/String LookAround`

##### Inputs:
  * (none)

##### Outputs
  * neck_euler - publishes neck angle
  * eyes - publishes eyes movements


### TrackDev
Eyes and head track a target, defined by a region of interest (ROI).
The system maintains multiple targets visible in the scene. The target
that will be tracked is selected with the /tracking_action topic.

For example:
   `rostopic pub --once /cmd_blendermode std_msgs/String TrackDev`

Each ROI is denoted with a cube.

Currently, the ROI's are human faces, extracted by pi_vision from a usb
video camera feed.  Multiple cameras can be supported by adding them to
src/config/inputs.yaml.  Each pi_vision node should run in it's own ROS
namespace.  The default (root node) camera is bound to chest_pivision
(configurable in inputs.yaml).  The eye namespace (export ROS_NAMESPACE=eye)
is currently bound to eye_pivision.

##### Topics subscribed:
  * /tracking_action (eva_behavior/tracking_action) - Listens for
    information on objects to track.


#### Inputs
  * chest_pivision (NRegionsOfInterest) - populates the scene with ROI
    from chest camera.
  * eye_pivision (NRegionsOfInterest) - populates the scene with ROI
    from eye camera.

##### Outputs
  * neck_euler - publishes neck angle.
  * eyes - publishes eyes movements.


### Animations
Plays pre-defined animations.  These are defined as object actions
within the blender rig.  The list of valid animations can be obtained
in two ways: using the old `/animations_list` topic or the new
`/blender_api/available_emotion_states` topic.  Neither topic list is
published until the rig is placed in animation mode.

Only the `animate-test.blend` rig supports Animations mode.

For example:
   `rostopic pub --once /cmd_blendermode std_msgs/String Animations`

##### Topics subscribed:
  * cmd_animations(std_msgs/String) - colon separated string which
    sends command (play,stop, pause) follwed optionaly the animation name.

    For example:
    `rostopic pub --once /cmd_animations std_msgs/String play:yawn-1`

##### Outputs:
  * full_head - publishes expression neck and eyes information.


### Manual Head
Used for animation development and debugging. Allows the designer
to control and move the rig, thus controlling the physical robot head.

##### Outputs:
  * full_head  - publishes expressions for neck and eyes positions.


### Dummy
Idle.

For example:
   `rostopic pub --once /cmd_blendermode std_msgs/String Dummy`

# Inputs
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

# Outputs
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


# Cookbook recipes

## Example LookAround Mode
The default rig demonstrates the LookAround mode.

```
	# Start blender, load head, start scripts
	blender ./robo_blender/src/robo.blend --enable-autoexec -P ./robo_blender/src/startup.py

	# Start the look-around mode
	rostopic pub --once /cmd_blendermode std_msgs/String LookAround

	# Verify that tracking output is sent to the PAU motors
	rostopic echo /arthur/cmd_eyes_pau
```

## Example TrackingDev Mode.
The TrackingDev mode requires that the [eva_behavior] ROS node be set
up and running.
```
	# Make sure all packages can be found.
	export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:... uvc_cam, pi_vision, etc.

	# Start up the video camera.  Use uvc_cam from hansonrobotics github.
	roslaunch uvc_cam uvc_cam.launch device:=/dev/video0

	# Verify camera is working
	rosrun image_view image_view image:=/camera/image_raw

	# Start pi_vision face tracker
	roslaunch pi_face_tracker face_tracker_uvc_cam.launch

	# Start blender, load head, start scripts
	blender ./robo_blender/src/robo.blend --enable-autoexec -P ./robo_blender/src/startup.py

	# Start the eva_behavior node
	rosrun eva_behavior general_behavior.py

	# Start the tracker
	rostopic pub /cmd_blendermode std_msgs/String TrackDev

	# XXX How to list the valid values for tracking_action?

	# XXX Set the roi to track.
	rostopic pub /tracking_action eva_behavior/tracking_action XXX ???

	# Verify that tracking output is sent to the PAU motors
	rostopic echo /arthur/cmd_eyes_pau
```
Example Multi-camera TrackingDev Mode. Same as above, except that new
cameras are now added:
```
	# Specify the 'eye' namespace
	export ROS_NAMESPACE=eye

	# Start up the video camera, but on a different vido device.
	roslaunch uvc_cam uvc_cam.launch device:=/dev/video1

	# Verify camera is working
	rosrun image_view image_view image:=/eye/camera/image_raw

	# Start pi_vision face tracker with the alternate camera
	# XXX this doesn't work ...
	roslaunch pi_face_tracker face_tracker_uvc_cam.launch input_rgb_image:=/eye/camera/image_raw

```

## Example Animations Mode

This requies a blender file that contains animations.  The animate-test
(another Arthur rig) will do.  The blender file should be kept in the
`./robo_blender/src/` directory, else the relative path names for
loading `main.py` will not work.

```
	# Start blender, load head, start scripts.
	blender ./robo_blender/src/animate-test.blend --enable-autoexec -P ./robo_blender/src/startup.py

	# Start the animations mode
	rostopic pub --once /cmd_blendermode std_msgs/String Animations

	# Obtain the list of supported animations. The result should show:
	# actions: ['angry-1', 'sad-1', 'happy-1', 'surprised-1']
	rostopic echo -n 1 /blender_api/available_emotion_states

	# Perform one of the supported animations
	rostopic pub --once /cmd_animations std_msgs/String play:sad-1

	# Verify that tracking output is sent to the PAU motors
	rostopic echo /arthur/cmd_eyes_pau

	# Pause the animation
	rostopic pub --once /cmd_animations std_msgs/String pause

	# Restart the animation
	rostopic pub --once /cmd_animations std_msgs/String play

	# Halt the animation
	rostopic pub --once /cmd_animations std_msgs/String stop
```
