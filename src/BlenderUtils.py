#!/usr/bin/env python3
import bpy
from mathutils import Matrix

def get_pose_matrix_in_other_space(mat, pose_bone):
  """
  Returns the transform matrix relative to pose_bone's current
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

  return smat

def get_local_pose_matrix(pose_bone):
  """Returns the local transform matrix of the given pose bone."""
  return get_pose_matrix_in_other_space(pose_bone.matrix, pose_bone)

def get_bones_rotation_rad(armature, bone, axis):
  mat = get_local_pose_matrix(bpy.data.objects[armature].pose.bones[bone])
  return getattr(mat.to_euler(), axis)