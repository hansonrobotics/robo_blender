import rospy
import bpy
import math

from mathutils import Matrix, Vector
from math import acos, degrees

from bpy.app.handlers import persistent
from std_msgs.msg import Float64

ang = 0
dynamixel_namespace = rospy.get_namespace()
rospy.init_node('blender_arm', anonymous=True)
dm = rospy.Publisher(dynamixel_namespace + 'tilt_controller/command', Float64)

def get_pose_matrix_in_other_space(mat, pose_bone):
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

def get_local_pose_matrix(pose_bone):
    """ Returns the local transform matrix of the given pose bone.
    """
    return get_pose_matrix_in_other_space(pose_bone.matrix, pose_bone)

def get_bones_rotation(armature,bone,axis):
    mat = get_local_pose_matrix(bpy.data.objects[armature].pose.bones[bone])
    if axis == 'z':
        return degrees(mat.to_euler().z)
    elif axis == 'y':
        return degrees(mat.to_euler().y)
    elif axis == 'x':
        return degrees(mat.to_euler().x)

def get_bones_rotation_rad(armature,bone,axis):
    mat = get_local_pose_matrix(bpy.data.objects[armature].pose.bones[bone])
    if axis == 'z':
        return mat.to_euler().z
    elif axis == 'y':
        return mat.to_euler().y
    elif axis == 'x':
        return mat.to_euler().x

@persistent
def load_handler(dummy):
    global ang
    global dm

    newang = get_bones_rotation_rad('Armature','bracket3','x')
    if ang != newang:
        ang = newang
        print("ANG: %s" % newang)
        dm.publish(float(newang))

bpy.app.handlers.scene_update_post.append(load_handler)

print("Started")
