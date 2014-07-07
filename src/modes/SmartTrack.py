import inputs
import sys
import bpy
import rospy
from basic_head_api.msg import MakeFaceExpr

class SmartTrack:

  class MovingTarget:
    target = None
    dest = None
    # Realitive point for the head in which we need to move objects
    offset = None
    #Speed is in radians per frame
    speed  = 0.01
    # variables for operation
    startLocation = None
    destLocation = None
    steps = 0
    currentStep = 0
    totalSteps = 0

    def __init__(self, target,destination,offset,speed):
      self.target = target
      self.dest = destination
      self.offset = offset
      if speed > 0 :
        self.speed = speed
      pass

    def move(self):
      diff = self.dest.location - self.target.location
      #ignore small differences in case
      if (diff.length  < 0.0001):
         return
      #start new movement if destination location is changed
      if (self.destLocation != self.dest.location) :
        self.destLocation = self.dest.location.copy()
        self.currentStep = 0
        #calculate total steps
        v1 = self.target.location - self.offset
        v2 = self.dest.location - self.offset
        ang = v1.angle(v2,0)
        self.totalSteps = max(int(ang/self.speed),1)
        self.startLocation =  self.target.location.copy()
      self.currentStep = self.currentStep +1
      # Check in case
      if self.currentStep > self.totalSteps:
        return

      v1 = self.dest.location - self.offset
      v2 = self.startLocation - self.offset
      v3 = v2.lerp(v1,self.currentStep/self.totalSteps)
      self.target.location = v3 + self.offset

  class Behavior:

    def __init__(self,fps,pub):
      #ticks synced with robo blender
      self.idle = 0
      self.current_person = 0
      self.current_emotion = None
      self.current_intensity = 0
      self.emotions = queue.Queue()
      #headmovement
      self.positions = queue.Queue()
      self.distance = 0.
      self.fps = fps
      self.currentTimer = 0
      self.randomEm = ["eureka","innocent","annoyed", "evil","horrified"]
      self.pub = pub

    def tick(self):
      facepos = inputs.get_instance("PiVision").location
      self.positions.put(facepos)
      while self.positions.qsize() >= self.fps:
        old = self.positions.get()
      diff = facepos - old
      self.distance = math.fabs(diff[0]) + math.fabs(diff[1]) + math.fabs(diff[2])

      if self.distance > 0.05:
        self.current_person += 1
        self.idle -= 1
        if self.current_person > 3:
          self.idle = 0
      else:
        self.idle += 1
        if self.idle > 10:
          self.current_person = 0
      # Top priority tasks:
      if self.current_person == 10:
        self.meet()
      if self.idle == 25:
        self.bye()
      self.process()


    def _em(self, emotion,intensity):
      msg = MakeFaceExpr()
      msg.exprname = emotion
      msg.intensity = intensity
      self.current_emotion = emotion
      self.current_intensity = intensity/11.
      self.pub.publish(msg)

    # Put all emotions to queue
    def _queue(self,emotion,intensity, timer):
      em = {'e': emotion, 'i':intensity, 't':timer}
      self.emotions.put(em)

    #Meet new face, do big smile
    def meet(self):
      self.emotions.queue.clear()
      self.currentTimer = 0
      self._queue('happy',11,50)
      self._queue('happy',10,10)
      self._queue('happy',9,10)
      self._queue('happy',8,10)


    def bye(self):
      self.emotions.queue.clear()
      self.currentTimer = 0
      self._queue('sad',7,50)
      self._queue('sad',8,10)
      self._queue('sad',9,10)
      self._queue('sad',10,60)

    def rand(self):
      em = self.randomEm[random.randrange(0,len(self.randomEm)-1)]
      i = random.randrange(7,11)
      timer = random.randrange(50,150)
      self._queue(em,i,timer)

    def next(self):
      ros.loginfo("current: ", self.current_person)
      # is meet and current face is detected
      if self.current_person > 10:
        if self.distance < 0.2 and self.current_intensity > 4:
          self._queue(self.current_emotion,self.current_intensity-1,25)
          return
        elif self.distance < 0.4:
          self._queue('surprised',8,25)
          return
        else:
          self._queue('surprised',11,25)
          return
      if self.idle > 10:
        if self.current_intensity > 4:
          self._queue(self.current_emotion,self.current_intensity-1,25)
      self.rand()

    def process(self):
      #no cahnge
      if self.currentTimer > 0:
        self.currentTimer -= 1
        return
      # need to do something
      if self.emotions.empty():
        self.next()
      # shouldnt be empty
      if self.emotions.empty():
        return
      em = self.emotions.get()
      self.currentTimer = em['t']
      self._em(em['e'],em['i'])

  def __init__(self):
    self.fps = 25
    self.behavior = Behavior(
      self.fps,
      rospy.Publisher(self.config['emo_topic'], MakeFaceExpr)
    )
    self.mTarget = MovingTarget(bpy.data.objects['headtarget'],bpy.data.objects['pivision'],bpy.data.objects['nose'].location,0.06)
    self.mEyes = MovingTarget(bpy.data.objects['eyefocus'],bpy.data.objects['pivision'],bpy.data.objects['nose'].location,0.09)
    self.leftover_time = 0.0

  def step(self, dt):
    # Execute tick() at the rate of self.fps
    dt = dt + self.leftover_time
    n = int(dt*self.fps)
    for i in range(n):
      self.tick()
    self.leftover_time = dt*self.fps - n

  def tick(self):
    self.behavior.tick()
    self.mTarget.move()
    self.mEyes.move()


instance = None
def init():
  instance = SmartTrack()
def step(dt):
  instance.step(dt)