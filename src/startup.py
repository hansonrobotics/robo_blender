#Copyright (c) 2013-2018 Hanson Robotics, Ltd.
import bpy
import os
import sys

dirname = os.path.dirname(bpy.data.filepath)
filename = os.path.join(dirname, "main.py")

sys.path.insert(0, dirname)
exec(compile(open(filename).read(), filename, 'exec'))