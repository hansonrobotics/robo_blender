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
        #Next playing animation. If not set animation stops
        self.init = False
        self.next = None
        self.command = None
        self.isPlaying = False

        self.current = None


    def parseCommand(self, msg):
        msg = msg.data
        data = msg.split(":", 2)
        command = data[0]
        #starts playing the animation, resumes current animation.
        # If animation is playing sets the next animation
        if command == 'play':
            self.command = 'play'
            self.next = data[1]
        #ends animation and stops playing
        if command == 'stop':
            self.command = 'stop'
            self.next = None
        #pause animation immediatly
        if command == 'pause':
            self.command = 'pause'

    #Sets next animation
    def _setNext(self):
        self.current = self.next
        self.anim.setAnimation(self.current)


    def step(self, dt):
        #Make sure we access bpy data and do other task in blender thread
        if not self.init:
            rospy.Subscriber('cmd_animations',String,self.parseCommand)
            self.animationsPub = rospy.Publisher('animations_list', animations_list,None, False,True,None,10)
            self.anim = animate.Animate('Armature')
            self.animationsList = self.anim.getAnimationList()
            self.animationsPub.publish(list(self.animationsList.keys()))
            self.init = True
            self.anim.resetAnimation()

        # Parse any pending commands if exists
        if self.command:
            if self.command == 'play':
                if not self.isPlaying:
                    if not self.current:
                        self._setNext()
                    self.isPlaying = True
                    self.anim.playAnimation()
            if self.command == 'pause':
                if self.isPlaying:
                    self.anim.stopAnimation()
                    self.isPlaying = False
            self.command = None

        if self.isPlaying:
            #animation finished
            rospy.logdebug(self.animationsList[self.current]['length'])
            rospy.logdebug(bpy.context.scene.frame_current)
            self.anim.playAnimation()
            if bpy.context.scene.frame_current > self.animationsList[self.current]['length']:
                if self.next:
                    self._setNext()
                    self.anim.playAnimation()
                else:
                    self.isPlaying = False
                    self.current = None
                    self.anim.resetAnimation()
            outputs.store.full_head.transmit()






