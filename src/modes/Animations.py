from .controllers import primary
from .controllers import animate
import inputs, outputs
import bpy
import rospy
from std_msgs.msg import String
from robo_blender.msg import animations_list

from blender_api_msgs.msg import AvailableEmotionStates
from blender_api_msgs.msg import AvailableGestures


class Animations:
    """
    Subscribes to the /cmd_animations topic, and listens for play,
    pause and stop messages. Queues up and starts/runs the animations
    appropriately.  Also published the list of supported animations
    at /animations_list.
    """
    def __init__(self):
        # Next playing animation. If not set animation stops
        self.init = False
        self.next = None
        self.command = None
        self.isPlaying = False

        self.current = None

    # Parse the old-style command string. We are expecting it to be
    # either  play:animation_name, stop or pause
    def parseCommand(self, msg):
        msg = msg.data
        data = msg.split(":", 2)
        command = data[0]

        # Start playing the animation, or resume the current animation.
        # If an animation is playing, then this sets the next animation.
        if command == 'play':
            self.command = 'play'

            # The animation name could be empty; that's OK.
            # Do not clobber the current animation, if a bogus
            # animation name was provided.
            if 1 < len(data):
                if not data[1] in self.animationsList:
                    rospy.logerr("Unknown animation: " + data[1])
                else:
                    self.next = data[1]
                    rospy.loginfo("Next animation: " + self.next)
            elif self.current:
                rospy.loginfo("Resume animation: " + self.current)

        # Halt animation and stops playing
        elif command == 'stop':
            self.command = 'stop'
            self.next = None
            rospy.loginfo("Stop animation")
            if not self.current:
                rospy.logwarn("Stop: no animation playing")

        # Pause animation immediatly
        elif command == 'pause':
            self.command = 'pause'
            rospy.loginfo("Pause animation")

        else:
            rospy.logerr("Unsupported command: " + command)

    # Sets next animation
    def _setNext(self):
        self.current = self.next

        # Don't crash if next animation is 'None'
        if self.current:
            self.anim.setAnimation(self.current)


    # This is called every frame
    def step(self, dt):
        # Make sure we access bpy data and do other task in blender thread
        if not self.init:
            self.anim = animate.Animate('Armature')
            self.animationsList = self.anim.getAnimationList()

            # Initialize the old-style ROS node
            rospy.Subscriber('cmd_animations', String, self.parseCommand)
            self.animationsPub = rospy.Publisher('animations_list',
                 animations_list, latch=True, queue_size=10)
            self.animationsPub.publish(list(self.animationsList.keys()))

            # Initialize the new blender_api_msgs ROS node
            self.emoPub = rospy.Publisher(
                '/blender_api/available_emotion_states',
                AvailableEmotionStates, latch=True, queue_size=10)
            self.emoPub.publish(list(self.animationsList.keys()))

            # XXX FIXME No gestures!?!
            self.gestPub = rospy.Publisher(
                '/blender_api/available_gestures',
                AvailableGestures, latch=True, queue_size=10)
            self.gestPub.publish(list())

            # Other initilizations
            self.init = True
            self.anim.resetAnimation()

        # Parse any pending commands
        if self.command:
            if self.command == 'play':
                if not self.isPlaying:
                    if not self.current:
                        self._setNext()
                    # If next was null, then, after above, current will
                    # become null, too.
                    if self.current:
                        self.isPlaying = True
                        self.anim.playAnimation()
                    else:
                        rospy.logwarn("Play: no pending animation to restart")

            elif self.command == 'pause':
                if self.isPlaying:
                    self.anim.stopAnimation()
                    self.isPlaying = False
                else:
                    rospy.logwarn("Pause: no animation playing, can't pause")
            self.command = None

        if self.isPlaying:

            rospy.logdebug(self.animationsList[self.current]['length'])
            rospy.logdebug(bpy.context.scene.frame_current)
            # self.anim.playAnimation()

            # Check to see if animation is done
            if bpy.context.scene.frame_current > self.animationsList[self.current]['length']:
                if self.next:
                    self._setNext()
                    self.anim.playAnimation()
                else:
                    self.isPlaying = False
                    self.current = None
                    self.anim.resetAnimation()
            outputs.store.full_head.transmit()

