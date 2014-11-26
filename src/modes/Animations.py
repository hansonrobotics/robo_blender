from .controllers import primary
from .controllers import animate
import inputs, outputs
import bpy
import rospy
from std_msgs.msg import String
from robo_blender.msg import animations_list



class Animations:
    """
    Transmits the current neck, face and eye position without actually
    controlling anything.
    """
    def __init__(self):
        # Next playing animation. If not set animation stops
        self.init = False
        self.next = None
        self.command = None
        self.isPlaying = False

        self.current = None


    # Parse the command string. We are expecting it to be either
    # play:animation_name, stop or pause
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
                    print("Error: Unknown animation: " + data[1])
                else:
                    self.next = data[1]

        # Halt animation and stops playing
        elif command == 'stop':
            self.command = 'stop'
            self.next = None

        # Pause animation immediatly
        elif command == 'pause':
            self.command = 'pause'
        else:
            print("Error: Unsupported command: " + command)

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
            rospy.Subscriber('cmd_animations',String,self.parseCommand)
            self.animationsPub = rospy.Publisher('animations_list', animations_list,None, False,True,None,10)
            self.anim = animate.Animate('Armature')
            self.animationsList = self.anim.getAnimationList()
            self.animationsPub.publish(list(self.animationsList.keys()))
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
            elif self.command == 'pause':
                if self.isPlaying:
                    self.anim.stopAnimation()
                    self.isPlaying = False
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

