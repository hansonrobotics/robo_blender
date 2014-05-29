#!/usr/bin/env python  
#import roslib; roslib.load_manifest('dmitry_tracker')
import rospy
import bpy
import math
import threading
import yaml

from mathutils import Matrix, Vector
from math import acos, degrees

from bpy.app.handlers import persistent
from std_msgs.msg import Float64
from std_msgs.msg import UInt16MultiArray

from ros_pololu_servo.msg import servo_pololu

class robo_blender :
    def read_motor_config(self, config):
        stream = open(config, 'r')
        self.config = yaml.load(stream)

    def send_motors(self):
        for motor in self.config:
            name = motor["name"]
            source = motor["source"].split(":")
            if source[0] == "shapekey":
                self.position_motor(motor, self.get_shape_position(motor))
            if source[0] == "bone":
                self.position_motor(motor, self.get_bone_position(motor))

    def get_bone_position(self, config):
        source = config["source"].split(":")
        bone_parent = source[1]
        bone = source[2]
        axis = source[3]
        cur_pos = self.get_bones_rotation_rad(bone_parent, bone, axis)
        #print("GET BONE: %s -> %s : %s = %s" % (bone_parent, bone, axis, cur_pos))
        return cur_pos

    def set_bone_position(self, config, position):
        source = config["source"].split(":")
        bone_parent = source[1]
        bone = source[2]
        axis = source[3]
        #print("SET BONE: %s -> %s : %s = %s" % (bone_parent, bone, axis, position))

    def get_shape_position(self, config):
        source = config["source"].split(":")
        shape_parent = source[1]
        shape = source[2]
        position = 0
        #print("GET SHAPE: %s -> %s = %s" % (shape_parent, shape, position))
        return position

    def set_shape_position(self, config, position):
        source = config["source"].split(":")
        shape_parent = source[1]
        shape = source[2]
        #print("SET SHAPE: %s -> %s = %s" % (shape_parent, shape, position))

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
            print("POLOLU: %s @ %s" % (config["name"], angle))
            pub.publish(msg)

    def position_dynamixel(self, config, angle):
        pub = self.dynamixels[config["name"]]
        if config["enabled"]: 
            print("DYNAMIXEL: %s @ %s" % (config["name"], angle))
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

        namespace = rospy.get_namespace()
        rospy.init_node('robo_blender', anonymous=True)

        # init pololu

        # init dynamixels
        for motor in self.config:
            if motor["type"] == "pololu" and motor["name"] not in self.pololus:
                self.pololus[motor["name"]] = rospy.Publisher(namespace + 'cmd_pololu', servo_pololu)
            if motor["type"] == "dynamixels" and motor["name"] not in self.dynamixels:
                self.dynamixels[motor["name"]] = rospy.Publisher(namespace + motor["ros_path"] + '/command', Float64)

        @persistent
        def load_handler(dummy):
            self.send_motors()

        bpy.app.handlers.scene_update_post.append(load_handler)

        print("ROBO: Started")

print("ROBO: Starting")
robo = robo_blender()
robo.read_motor_config("motors.yaml")
robo.execute()
