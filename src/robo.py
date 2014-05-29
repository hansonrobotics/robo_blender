#!/usr/bin/env python  
#import roslib; roslib.load_manifest('dmitry_tracker')
import rospy
import bpy
import math
import threading

from mathutils import Matrix, Vector
from math import acos, degrees

from bpy.app.handlers import persistent
from std_msgs.msg import Float64
from std_msgs.msg import UInt16MultiArray

from ros_pololu_servo.msg import servo_pololu

class robo_blender :
    def position_motor(self, motor, angle):
        #msg = servo_pololu()
        #msg.id = 1
        #msg.angle = newreye
        #msg.speed = 0
        #msg.acceleration = 0
        #self.pololu.publish(msg)
        print("MOTOR: %s %s" % (motor, angle))

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
        self.doeyes = True
        self.domotors = False
        self.leye = 0
        self.reye = 0
        self.jaw = 0
        self.neck0 = 0
        self.neck1 = 0
        self.neck2 = 0
        self.neck3 = 0

        namespace = rospy.get_namespace()
        rospy.init_node('blender_arm', anonymous=True)

        if self.doeyes:
            self.pololu = rospy.Publisher(namespace + 'cmd_pololu', servo_pololu)

        if self.domotors: 
            self.jawdm = rospy.Publisher(namespace + 'jaw/command', Float64)
            self.neck0dm = rospy.Publisher(namespace + 'neck0/command', Float64)
            self.neck1dm = rospy.Publisher(namespace + 'neck1/command', Float64)
            self.neck2dm = rospy.Publisher(namespace + 'neck2/command', Float64)
            self.neck3dm = rospy.Publisher(namespace + 'neck3/command', Float64)

        def faceCallback(thearray):
            facemarker = bpy.data.objects['facedetect']
            facemarker.location.x = 1.5 - (thearray.data[0] / 100) 
            facemarker.location.z = 1.5 - (thearray.data[1] / 100)
            print("CALLBACK %s" % thearray)

        facedetect = rospy.Subscriber('/facedetect', UInt16MultiArray, faceCallback)

        @persistent
        def load_handler(dummy):
            newstuff = False

            newleye = self.get_bones_rotation_rad('leye','leyebone','z')
            if self.leye != newleye:
                self.leye = newleye
                eyedeg = self.get_bones_rotation('leye','leyebone','z')
                if self.doeyes: 
                    msg = servo_pololu()
                    msg.id = 0
                    msg.angle = newleye
                    msg.speed = 0
                    msg.acceleration = 0
                    self.pololu.publish(msg)
                    print("LEYE: %s" % msg)

            newreye = self.get_bones_rotation_rad('reye','reyebone','z')
            if self.reye != newreye:
                self.reye = newreye
                eyedeg = self.get_bones_rotation('reye','reyebone','z')
                if self.doeyes: 
                    msg = servo_pololu()
                    msg.id = 1
                    msg.angle = newreye
                    msg.speed = 0
                    msg.acceleration = 0
                    self.pololu.publish(msg)
                    print("REYE: %s" % msg)

            newjaw = self.get_bones_rotation_rad('jawarm','jawbone','x')
            if self.jaw != newjaw:
                self.jaw = newjaw
                jawdeg = self.get_bones_rotation('jawarm','jawbone','x')
                if self.domotors: self.jawdm.publish(float(jaw))
                print("JAW: %s (%s)" % (self.jaw, jawdeg))

            newneck0 = self.get_bones_rotation_rad('Armature','base','y') + 2.2
            if self.neck0 != newneck0:
                self.neck0 = newneck0
                neck0deg = self.get_bones_rotation('Armature','base','y')
                if self.domotors: self.neck0dm.publish(float(newneck0))
                print("BASE: %s (%s)" % (self.neck0, neck0deg))

            newneck1 = (self.get_bones_rotation_rad('Armature','bracket1','x') * -2) + 2.5
            if self.neck1 != newneck1:
                self.neck1 = newneck1
                if self.domotors: self.neck1dm.publish(float(newneck1))
                print("NECK1: %s" % self.neck1)

            newneck2 = (self.get_bones_rotation_rad('Armature','bracket2','z') * 2) + 2
            if self.neck2 != newneck2:
                self.neck2 = newneck2
                if self.domotors: self.neck2dm.publish(float(newneck2))
                print("NECK2: %s" % self.neck2)

            newneck3 = (self.get_bones_rotation_rad('Armature','bracket3','x') * 2) + 3
            if self.neck3 != newneck3:
                self.neck3 = newneck3
                if self.domotors: self.neck3dm.publish(float(newneck3))
                print("NECK3: %s" % self.neck3)

        bpy.app.handlers.scene_update_post.append(load_handler)

        print("ROBO: Started")

print("ROBO: Starting")
robo = robo_blender()
robo.execute()
