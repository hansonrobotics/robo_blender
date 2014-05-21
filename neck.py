import rospy
import bpy
import math

from bpy.app.handlers import persistent
from std_msgs.msg import Float64

xaxis = 0
dynamixel_namespace = rospy.get_namespace()
rospy.init_node('blender_arm', anonymous=True)
dm = rospy.Publisher(dynamixel_namespace + 'tilt_controller/command', Float64)

@persistent
def load_handler(dummy):
    global xaxis
    global dm
    if xaxis != bpy.data.objects['Armature'].pose.bones['bracket3'].x_axis:
        xaxis = bpy.data.objects['Armature'].pose.bones['bracket3'].x_axis
        angle = math.atan2(xaxis[1], xaxis[0])
        print("ANGLE:%s" % angle)
        dm.publish(float(angle))

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


bpy.app.handlers.scene_update_post.append(load_handler)

print("Started")
