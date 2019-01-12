#!/usr/bin/env python3
#Copyright (c) 2013-2018 Hanson Robotics, Ltd.
import yaml
import os
import importlib
import genpy
import roslib.message
import copy
import re

class DictKeyChain:
  """
  Allows to build a chain of dictionary keys (or array indices or object
  attributes), which can be later applied to an obj.

  I.e. a DictKeyChain instance constructed with key_list ['foo', 'bar'], can
  be applied on a dictionary {'foo': {'bar': 'myStr'}} to get the 'myStr'
  string.
  """

  @staticmethod
  def _get_from(host, key_list):
    for key in key_list:
      if hasattr(host, "__getitem__"):
        host = host[key]
      else:
        host = getattr(host, key)
    return host

  def get_from(self, host):
    return self._get_from(host, self.key_list)

  def set_on(self, host, val):
    host = self._get_from(host, self.key_list[:-1])
    key = self.key_list[-1]
    if hasattr(host, "__setitem__"):
      host[key] = val
    else:
      setattr(host, key, val)

  def __init__(self, key_list):
    # Parse strings to integers where possible
    self.key_list = list(map(
      lambda key: int(key)
        if isinstance(key, str) and key.isdigit()
        else key,
      key_list
    ))

class RosMsgKeyChain(DictKeyChain):

  list_brackets = re.compile(r'\[[^\]]*\]$')

  @classmethod
  def _get_ros_arr_class(cls, arr_host, key):
    #E.g. trajectory_msgs/JointTrajectoryPoint[]
    arr_type = dict(zip(arr_host.__slots__, arr_host._slot_types))[key]
    #E.g. trajectory_msgs/JointTrajectoryPoint
    entry_type = cls.list_brackets.sub('', arr_type)
    return roslib.message.get_message_class(entry_type)

  @classmethod
  def _hard_get_from(cls, host, key_list):
    for key in key_list:
      if isinstance(host, list):
        key = int(key)
        if key > len(host):
          raise IndexError("Index %d of %s is out of bounds." % key, host)
        elif key == len(host):
          # We can find what class is appropriate to append to this list by
          # looking at the parent of this list.
          host.append(cls._get_ros_arr_class(*prev_hostkey)())

      prev_hostkey = (host, key)
      if isinstance(host, list):
        host = host[key]
      else:
        host = getattr(host, key)
    return host

  @staticmethod
  def _hard_set_on(host, key, val):
    if isinstance(host, list):
      key = int(key)
      if key > len(host):
        raise IndexError("Index %d of %s is out of bounds." % key, host)
      elif key == len(host):
        host.append(val)
      else:
        host[key] = val
    else:
      setattr(host, key, val)

  def hard_set_on(self, host, val):
    """
    Behaves like set_on(self, host, val), except this method creates necessary
    nested ROS messages on its way to the end of the keychain.

    E.g. Key chain ["points", "0", "positions"] applied with this method to an
    empty JointTrajectory instance would append the empty "points" list with a
    JointTrajectoryPoint instance and set its "positions" attribute to the
    given val.
    """
    host = self._hard_get_from(host, self.key_list[:-1])
    key = self.key_list[-1]
    self._hard_set_on(host, key, val)


def read_yaml(yamlname, loader_filepath=None):
  """
  Pass __file__ as loader_filepath argument to load file relative to the
  caller file location.
  """
  if not loader_filepath:
    loader_filepath = __file__
  dirname, filename = os.path.split(os.path.abspath(loader_filepath))

  stream = open(os.path.join(dirname, yamlname), 'r')
  result = yaml.load(stream)
  stream.close()
  return result

def import_member(name):
  """
  Behaves like importlib.import_module except that this takes a path
  to a member (a class or a function) inside a module, instead of a 
  path to a module.
  """
  path = name.split(".")
  module = importlib.import_module((".".join(path[:-1])))
  return getattr(module, path[-1])
