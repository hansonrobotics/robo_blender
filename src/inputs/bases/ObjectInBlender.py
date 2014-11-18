import bpy
from . import InputBase
from mathutils import Vector

class ObjectSelection:
  """
  Saves and restores selection (including the active object) to allow bpy
  operations without disturbing user's perceived selection.
  """

  def __init__(self):
    active = bpy.context.scene.objects.active
    self.active_name = None if active == None else active.name
    self.selected_names = [obj.name for obj in bpy.context.selected_objects]

  def restore(self):
    bpy.ops.object.select_all(action="DESELECT")
    bpy.context.scene.objects.active = (
      bpy.data.objects[self.active_name]
      if self.active_name != None and self.active_name in bpy.context.scene.objects
      else None
    )
    for name in self.selected_names:
      bpy.ops.object.select_pattern(pattern=name)

class ObjectInBlender(InputBase):
  """
  A base class that represents a sensory input as an object in Blender space.
  """

  #The name of the desired parent object
  GROUP_NAME = "inputs"

  def __init__(self, confentry):
    self.confentry = confentry
    self.binding = confentry["binding"]["objectpos"]
    self._location = Vector()
    self._update_from_object()

  @classmethod
  def from_binding(cls, binding):
    return cls({"enabled": True, "binding": binding})

  def destroy(self):
    if not self.binding["name"] in bpy.data.objects:
      return
    #Backup current selection
    selection = ObjectSelection()
    #Delete binded object
    bpy.ops.object.select_all(action="DESELECT")
    bpy.data.objects[self.binding["name"]].select = True
    bpy.ops.object.delete()
    #Restore selection
    selection.restore()

  def set_object_location(self, point):
    self._confirm_object(self.binding["name"])
    # Go straight to _set_object_location() next time.
    self.set_object_location = self._set_object_location
    self.set_object_location(point)

  @property
  def location(self):
    self._update_from_object()
    return self._location

  def _set_object_location(self, point):
    if self.confentry["enabled"]:
      offset = None
      if isinstance(self.binding["offset"], list):
        offset = Vector(self.binding["offset"])
      else:
        offset = bpy.data.objects[self.binding["offset"]].matrix_world.to_translation()
        # Check if vector and distance is set
        if 'direction' in self.binding:
          distance = 1
          if 'distance' in self.binding:
            distance = self.binding['distance']
          d = bpy.data.objects[self.binding["direction"]].location-offset
          d.normalize()
          offset = offset + d * distance
      self._location = offset + point * self.binding["scale"]
      bpy.data.objects[self.binding["name"]].location = self._location

  @classmethod
  def _confirm_group(cls):
    """Creates an empty object to put the input objects in, if it's not there."""
    if cls.GROUP_NAME in bpy.data.objects:
      return
    #Backup current selection
    selection = ObjectSelection()
    #Create empty object
    bpy.ops.object.empty_add()
    new_group = bpy.context.selected_objects[0]
    new_group.name = cls.GROUP_NAME
    new_group.hide = True
    #Restore selection
    selection.restore()

  @classmethod
  def _confirm_object(cls, name):
    """Create blender object with the specified name if it's not there."""
    if name in bpy.data.objects:
      return
    cls._confirm_group()
    #Backup current selection
    selection = ObjectSelection()
    #Create cube
    bpy.ops.mesh.primitive_cube_add(radius=0.024)
    new_cube = bpy.context.selected_objects[0]
    new_cube.parent = bpy.data.objects[cls.GROUP_NAME]
    new_cube.name = name
    cls._hash_color(new_cube)
    #Restore selection
    selection.restore()

  @staticmethod
  def _hash_color(obj):
    """ Color object by its first two letters from its name. """
    name_hash = hash(obj.name[:2])
    color = (
      (name_hash >> 16) % 256,
      (name_hash >> 8) % 256,
      name_hash % 256
    )
    mat_name = "#%02X%02X%02X" % color
    mat = (
      bpy.data.materials[mat_name] if mat_name in bpy.data.materials
      else bpy.data.materials.new(mat_name)
    )
    mat.diffuse_color = tuple([i / 256 for i in color])
    obj.data.materials.append(mat)

  def _update_from_object(self):
    pass
    if self.binding["name"] in bpy.data.objects:
      self._location = bpy.data.objects[self.binding["name"]].location
