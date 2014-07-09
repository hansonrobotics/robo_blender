#!/usr/bin/env python3
import yaml
import os
import importlib

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
    return self._get_from(host, key_list)

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
  Behaves like importlib.import_module except takes a path to a member (a class or a
  function) inside a module, instead of a path to a module.
  """
  path = name.split(".")
  module = importlib.import_module((".".join(path[:-1])))
  return getattr(module, path[-1])