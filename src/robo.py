#!/usr/bin/env python  
#import roslib; roslib.load_manifest('dmitry_tracker')
import rospy
import bpy
import math
import threading
import yaml
import importlib
import pprint

from mathutils import *
from math import acos, degrees

from bpy.app.handlers import persistent
# import standard messages types
from std_msgs.msg import *
from sensor_msgs.msg import *
from ros_pololu_servo.msg import servo_pololu
from ros_faceshift.msg import *
from basic_head_api.msg import PointHead


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
        if (diff.length  < 0.001):
            return
        #start new movement if destination location is changed
        if (self.destLocation != self.dest.location) :
            self.destLocation = self.dest.location.copy()
            self.currentStep = 0
            #calculate total steps
            v1 = self.target.location - self.offset
            v2 = self.dest.location - self.offset
            ang = v1.angle(v2,0)
            self.totalSteps = int(ang/self.speed)
            self.startLocation =  self.target.location.copy()
        self.currentStep = self.currentStep +1
        # Check in case
        if self.currentStep > self.totalSteps:
            return

        v1 = self.dest.location - self.offset
        v2 = self.startLocation - self.offset
        v3 = v2.lerp(v1,self.currentStep/self.totalSteps)
        self.target.location = v3 + self.offset


class robo_blender :
    def read_motor_config(self, config):
        stream = open(config, 'r')
        self.config = yaml.load(stream)

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
                    print(src)
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

    def send_motors(self):
        for motor in self.config:
            name = motor["name"]
            binding = motor["binding"].split(":")
            if binding[0] == "shapekey":
                self.position_motor(motor, self.get_shape_position(motor))
            if binding[0] == "bone":
                self.position_motor(motor, self.get_bone_position(motor))

    def get_bone_position(self, config):
        binding = config["binding"].split(":")
        bone_parent = binding[1]
        bone = binding[2]
        axis = binding[3]
        cur_pos = self.get_bones_rotation_rad(bone_parent, bone, axis)
        #print("GET BONE: %s -> %s : %s = %s" % (bone_parent, bone, axis, cur_pos))
        return cur_pos

    def get_head_rotation_euler(self):
        msg = PointHead()
        msg.pitch = self.get_bone_position(self.bones['headpitch'])
        msg.roll = self.get_bone_position(self.bones['headroll'])
        msg.yaw = 0-self.get_bone_position(self.bones['headrotation'])
        return msg


    def send_head_rotion(self):
        msg = self.get_head_rotation_euler()
        self.point_head.publish(msg)

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
        binding = config["binding"].split(":")
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

    def position_motor(self, config, angle):
        #print("POSITION %s" % angle)
        if config["name"] in self.positions:
            cur_pos = self.positions[config["name"]]
            if cur_pos != angle:
                if config["type"] == "pololu": self.position_pololu(config, angle)
                if config["type"] == "dynamixel": self.position_pololu(config, angle)
        self.positions[config["name"]] = angle

    def position_pololu(self, config, angle):
        msg = servo_pololu()
        msg.id = int(config["motorid"])
        msg.angle = float(angle * float(config["scale"]) + float(config["translate"]))
        msg.speed = int(config["speed"])
        msg.acceleration = int(config["acceleration"])
        pub = self.pololus[config["name"]]
        if config["enabled"]: 
            #print("POLOLU: %s @ %s" % (config["name"], angle))
            pub.publish(msg)

    def position_dynamixel(self, config, angle):
        pub = self.dynamixels[config["name"]]
        if config["enabled"]: 
            #print("DYNAMIXEL: %s @ %s" % (config["name"], angle))
            pub.publish(float(angle))

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
        self.dynamixels = {}
        self.pololus = {}

        self.ops = []

        namespace = rospy.get_namespace()
        rospy.init_node('robo_blender', anonymous=True)

        # init pololu

        # init dynamixels
        for motor in self.config:
            if motor["type"] == "pololu" and motor["name"] not in self.pololus:
                self.pololus[motor["name"]] = rospy.Publisher(namespace + 'cmd_pololu', servo_pololu)
            if motor["type"] == "dynamixels" and motor["name"] not in self.dynamixels:
                self.dynamixels[motor["name"]] = rospy.Publisher(namespace + motor["ros_path"] + '/command', Float64)
        self.point_head = rospy.Publisher(namespace+'point_head',PointHead)
        # start target
        self.mTarget = MovingTarget(bpy.data.objects['target'],bpy.data.objects['destination'],bpy.data.objects['nose'].location,0.005)
        self.mEyes = MovingTarget(bpy.data.objects['eyefocus'],bpy.data.objects['destination'],bpy.data.objects['nose'].location,0.04)


        @persistent
        def load_handler(dummy):

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
            #self.send_motors()
            self.send_head_rotion()
            self.mTarget.move()
            self.mEyes.move()
        bpy.app.handlers.scene_update_pre.append(load_handler)
        print("ROBO: Started")




print("ROBO: Starting")
robo = robo_blender()
robo.read_motor_config("motors.yaml")
robo.read_input_config("inputs.yaml")
robo.execute()
