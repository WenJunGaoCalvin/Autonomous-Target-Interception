# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: world_stats.proto

import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


import log_playback_stats_pb2 as log__playback__stats__pb2
import time_pb2 as time__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='world_stats.proto',
  package='gazebo.msgs',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=_b('\n\x11world_stats.proto\x12\x0bgazebo.msgs\x1a\x18log_playback_stats.proto\x1a\ntime.proto\"\xfc\x01\n\x0fWorldStatistics\x12#\n\x08sim_time\x18\x02 \x02(\x0b\x32\x11.gazebo.msgs.Time\x12%\n\npause_time\x18\x03 \x02(\x0b\x32\x11.gazebo.msgs.Time\x12$\n\treal_time\x18\x04 \x02(\x0b\x32\x11.gazebo.msgs.Time\x12\x0e\n\x06paused\x18\x05 \x02(\x08\x12\x12\n\niterations\x18\x06 \x02(\x04\x12\x13\n\x0bmodel_count\x18\x07 \x01(\x05\x12>\n\x12log_playback_stats\x18\x08 \x01(\x0b\x32\".gazebo.msgs.LogPlaybackStatistics')
  ,
  dependencies=[log__playback__stats__pb2.DESCRIPTOR,time__pb2.DESCRIPTOR,])




_WORLDSTATISTICS = _descriptor.Descriptor(
  name='WorldStatistics',
  full_name='gazebo.msgs.WorldStatistics',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='sim_time', full_name='gazebo.msgs.WorldStatistics.sim_time', index=0,
      number=2, type=11, cpp_type=10, label=2,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='pause_time', full_name='gazebo.msgs.WorldStatistics.pause_time', index=1,
      number=3, type=11, cpp_type=10, label=2,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='real_time', full_name='gazebo.msgs.WorldStatistics.real_time', index=2,
      number=4, type=11, cpp_type=10, label=2,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='paused', full_name='gazebo.msgs.WorldStatistics.paused', index=3,
      number=5, type=8, cpp_type=7, label=2,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='iterations', full_name='gazebo.msgs.WorldStatistics.iterations', index=4,
      number=6, type=4, cpp_type=4, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='model_count', full_name='gazebo.msgs.WorldStatistics.model_count', index=5,
      number=7, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='log_playback_stats', full_name='gazebo.msgs.WorldStatistics.log_playback_stats', index=6,
      number=8, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=73,
  serialized_end=325,
)

_WORLDSTATISTICS.fields_by_name['sim_time'].message_type = time__pb2._TIME
_WORLDSTATISTICS.fields_by_name['pause_time'].message_type = time__pb2._TIME
_WORLDSTATISTICS.fields_by_name['real_time'].message_type = time__pb2._TIME
_WORLDSTATISTICS.fields_by_name['log_playback_stats'].message_type = log__playback__stats__pb2._LOGPLAYBACKSTATISTICS
DESCRIPTOR.message_types_by_name['WorldStatistics'] = _WORLDSTATISTICS
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

WorldStatistics = _reflection.GeneratedProtocolMessageType('WorldStatistics', (_message.Message,), dict(
  DESCRIPTOR = _WORLDSTATISTICS,
  __module__ = 'world_stats_pb2'
  # @@protoc_insertion_point(class_scope:gazebo.msgs.WorldStatistics)
  ))
_sym_db.RegisterMessage(WorldStatistics)


# @@protoc_insertion_point(module_scope)
