#!/usr/bin/env python3
#import roslib; roslib.load_manifest('dmitry_tracker')
import rospy
import bpy
import math
import threading
import yaml
import importlib
import pprint
import time
import queue
from mathutils import *
from math import acos, degrees
import random

from bpy.app.handlers import persistent
# import standard messages types
from std_msgs.msg import *
from sensor_msgs.msg import *
from ros_pololu_servo.msg import servo_pololu
from ros_faceshift.msg import *
from basic_head_api.msg import PointHead
from pau2motors.msg import fsMsgTrackingState
from basic_head_api.msg import MakeFaceExpr


class MovingTarget :
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

#Demo to control robots emotions based on faces
class Behavior :

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


    def tick(self, ops):
        #currently we use only face detection
        if len(ops)>0:
            self.current_person += 1
            self.idle -= 1
            if self.current_person > 3:
                self.idle = 0
            last = ops[-1]
            msg = last[1]
            self.positions.put(msg)
            if self.positions.qsize() >= self.fps:
                old = self.positions.get()
                self.distance = math.fabs(old.x_offset+old.width/2 -msg.x_offset - msg.width/2.)\
                                +math.fabs(old.y_offset+old.height/2 -msg.y_offset -msg.height/2)
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
        print ("current: ", self.current_person)
        # is meet and current face is detected
        if self.current_person > 10:
            if self.distance < 100 and self.current_intensity > 4:
                self._queue(self.current_emotion,self.current_intensity-1,25)
                return
            if self.distance >= 100 and self.distance < 200:
                self._queue('surprised',8,25)
                return
            if self.distance > 200:
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


class robo_blender :
    def read_config(self, config):
        stream = open(config, 'r')
        self.config = yaml.load(stream)

    def read_pau_config(self, config):
        stream = open(config, 'r')
        self.pau = yaml.load(stream)

    def get_head_rotation_euler(self):
        msg = PointHead()
        msg.pitch = self.get_bone_position(self.bones['headpitch'])
        msg.roll = self.get_bone_position(self.bones['headroll'])
        msg.yaw = 0-self.get_bone_position(self.bones['headrotation'])
        return msg


    def send_head_rotion(self):
        msg = self.get_head_rotation_euler()
        self.point_head.publish(msg)

    def read_input_config(self, config):
        stream = open(config, 'r')
        self.inputs = yaml.load(stream)
        processors = {}
        source_listeners = {}
        self.bones = {}

        for con in self.inputs:
            name = con["name"]
            self.bones[con["name"]] = con
            binding = con["binding"].split(":")
            source = con["source"].split(":")
            if con["processor"] not in processors:
                filename = os.path.join(os.path.dirname(bpy.data.filepath), "processors/%s.py" % con["processor"])
                exec(compile(open(filename).read(), "processors/%s.py" % con["processor"], 'exec'), globals(), locals())
                processor = processors[con["processor"]]

            if source[0] not in source_listeners:
                source_listeners[source[0]] = []

                def call_listeners(msg,src):
                    for callback_pair in source_listeners[src]:
                        listener = callback_pair[0]
                        con = callback_pair[1]
                        #print("CALLING %s %s" % (con["source"], con["binding"]))
                        listener(con, msg)

                def bind(src):
                    print("BINDING %s" % src)
                    rospy.Subscriber(src, processor.msgclass(), lambda msg : call_listeners(msg,src))

                bind(source[0])

            def callback(con, msg):
                self.ops.append([processor, msg, con])

            source_listeners[source[0]].append([callback, con])
            print("ADDING %s" % con["source"])

    def send_pau(self):
        pau_msg = fsMsgTrackingState()
        # head rotation
        e = Euler((0,0,0))
        #set 0 for coeffs
        pau_msg.m_coeffs = [0]*48;
        for config in self.pau:
            source = config["source"].split(":")
            if source[0] == "shapekey":
                val = self.get_shape_position(config)
            if source[0] == "bone":
                val = self.get_bone_position(config)
            val = val*config['scale']+config['translate']

            msgKey = config["pau"].split(":")
            if msgKey[0] == "m_coeffs":
                pau_msg.m_coeffs[int(msgKey[1])] = val
            if msgKey[0] == "m_headRotation":
                setattr(e,msgKey[1],val)
            if msgKey[0] == "m_headTranslation":
                setattr(pau_msg.m_headTranslation,msgKey[1],val)
        #Euler to quaternion
        q = e.to_quaternion()
        pau_msg.m_headRotation.x = q.x
        pau_msg.m_headRotation.y = q.y
        pau_msg.m_headRotation.z = q.z
        pau_msg.m_headRotation.w = q.w
        self.pau_pub.publish(pau_msg)


    def get_bone_position(self, config):
        binding = config["binding"].split(":")
        bone_parent = binding[1]
        bone = binding[2]
        axis = binding[3]
        cur_pos = self.get_bones_rotation_rad(bone_parent, bone, axis)
        #print("GET BONE: %s -> %s : %s = %s" % (bone_parent, bone, axis, cur_pos))
        return cur_pos


    def set_bone_position(self, config, position):
        binding = config["binding"].split(":")
        bone_parent = binding[1]
        bone = binding[2]
        axis = binding[3]
        bpy.data.objects[bone_parent].pose.bones[bone].rotation_mode = 'XYZ'
        rot = bpy.data.objects[bone_parent].pose.bones[bone].rotation_euler

#        print("AXIS %s" % axis)

        if axis == "x":
            rot[0] = position * -1
        if axis == "y":
            rot[1] = position
        if axis == "z":
            rot[2] = position
        bpy.data.objects[bone_parent].pose.bones[bone].rotation_euler = rot
        #print("SET BONE: %s -> %s : %s = %s" % (bone_parent, bone, axis, rot))
        
    def get_shape_position(self, config):
        binding = config["source"].split(":")
        shape_parent = binding[1]
        shape = binding[2]
        position = bpy.data.meshes[shape_parent].shape_keys.key_blocks[shape].value
        #print("GET SHAPE: %s -> %s = %s" % (shape_parent, shape, position))
        return position

    def set_shape_position(self, config, position):
        if config["enabled"]:
            binding = config["binding"].split(":")
            shape_parent = binding[1]
            shape = binding[2]
            scale = config["scale"]
            translate = config["translate"]
            #print("SET SHAPE: %s -> %s = %s" % (shape_parent, shape, position))
            if config["invert"]: position = 1 - position
            bpy.data.meshes[shape_parent].shape_keys.key_blocks[shape].value = position * scale

    def set_object_position(self, config, position):
        if config["enabled"]:
            binding = config["binding"].split(":")
            object = binding[1]
            bpy.data.objects[object].location = Vector(config["offset"])+position*config["scale"]
            #pprint.pprint(config["offset"])

    def get_pose_matrix_in_other_space(self,mat, pose_bone):
        """ Returns the transform matrix relative to pose_bone's current
        transform space. In other words, presuming that mat is in
        armature space, slapping the returned matrix onto pose_bone
        should give it the armature-space transforms of mat.
        TODO: try to handle cases with axis-scaled parents better.
        """
        rest = pose_bone.bone.matrix_local.copy()
        rest_inv = rest.inverted()
        if pose_bone.parent:
            par_mat = pose_bone.parent.matrix.copy()
            par_inv = par_mat.inverted()
            par_rest = pose_bone.parent.bone.matrix_local.copy()
        else:
            par_mat = Matrix()
            par_inv = Matrix()
            par_rest = Matrix()

        # Get matrix in bone's current transform space
        smat = rest_inv * (par_rest * (par_inv * mat))

        # Compensate for non-local location
        #if not pose_bone.bone.use_local_location:
        # loc = smat.to_translation() * (par_rest.inverted() * rest).to_quaternion()
        # smat.translation = loc

        return smat

    def get_local_pose_matrix(self,pose_bone):
        """ Returns the local transform matrix of the given pose bone.
        """
        return self.get_pose_matrix_in_other_space(pose_bone.matrix, pose_bone)

    def get_bones_rotation(self,armature,bone,axis):
        mat = self.get_local_pose_matrix(bpy.data.objects[armature].pose.bones[bone])
        if axis == 'z':
            return degrees(mat.to_euler().z)
        elif axis == 'y':
            return degrees(mat.to_euler().y)
        elif axis == 'x':
            return degrees(mat.to_euler().x)

    def get_bones_rotation_rad(self,armature,bone,axis):
        mat = self.get_local_pose_matrix(bpy.data.objects[armature].pose.bones[bone])
        if axis == 'z':
            return mat.to_euler().z
        elif axis == 'y':
            return mat.to_euler().y
        elif axis == 'x':
            return mat.to_euler().x

    def execute(self):
        self.positions = {}

        self.ops = []
        self.lastframe = time.time()
        self.frame_interval = 1/self.config["fps"]
        namespace = rospy.get_namespace()
        rospy.init_node('robo_blender', anonymous=True)

        # PAU publisher
        self.pau_pub = rospy.Publisher(namespace + self.config['pau_topic'], fsMsgTrackingState)
        self.point_head = rospy.Publisher(namespace+'point_head',PointHead)
        #Emotional behaviour
        self.emo_pub = rospy.Publisher(namespace + self.config['emo_topic'], MakeFaceExpr)
        self.behaviour =  Behavior(self.config["fps"],self.emo_pub )
        # start target
        self.mTarget = MovingTarget(bpy.data.objects['target'],bpy.data.objects['destination'],bpy.data.objects['nose'].location,0.06)
        self.mEyes = MovingTarget(bpy.data.objects['eyefocus'],bpy.data.objects['destination'],bpy.data.objects['nose'].location,0.09)


        @persistent
        def load_handler(dummy):
            t = time.time()
            if t - self.lastframe < 1/self.config["fps"]:
                return
            self.lastframe = self.lastframe + self.frame_interval
            #control emotion
            self.behaviour.tick(self.ops)
            for op in self.ops:
                processor = op[0]
                msg = op[1]
                con = op[2]
                binding = con["binding"].split(":")
                r = processor.process(msg, con)
                #if binding[0] == "shapekey":
                    #self.set_shape_position(con, r)
                #if binding[0] == "bone":
                    #self.set_bone_position(con, r)
                if binding[0] == "object_pos":
                    self.set_object_position(con, r)

            self.ops = []
            self.send_head_rotion()
            #self.send_pau()
            self.mTarget.move()
            self.mEyes.move()
        bpy.app.handlers.scene_update_pre.append(load_handler)
        print("ROBO: Started")
print("ROBO: Starting")
robo = robo_blender()
robo.read_config("config.yaml")
robo.read_input_config("inputs.yaml")
robo.read_pau_config("pau.yaml")
robo.execute()
