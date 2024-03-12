# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: wrench_stamped.proto

import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


import time_pb2 as time__pb2
import wrench_pb2 as wrench__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='wrench_stamped.proto',
  package='gazebo.msgs',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=_b('\n\x14wrench_stamped.proto\x12\x0bgazebo.msgs\x1a\ntime.proto\x1a\x0cwrench.proto\"U\n\rWrenchStamped\x12\x1f\n\x04time\x18\x01 \x02(\x0b\x32\x11.gazebo.msgs.Time\x12#\n\x06wrench\x18\x02 \x02(\x0b\x32\x13.gazebo.msgs.Wrench')
  ,
  dependencies=[time__pb2.DESCRIPTOR,wrench__pb2.DESCRIPTOR,])




_WRENCHSTAMPED = _descriptor.Descriptor(
  name='WrenchStamped',
  full_name='gazebo.msgs.WrenchStamped',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='time', full_name='gazebo.msgs.WrenchStamped.time', index=0,
      number=1, type=11, cpp_type=10, label=2,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='wrench', full_name='gazebo.msgs.WrenchStamped.wrench', index=1,
      number=2, type=11, cpp_type=10, label=2,
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
  serialized_start=63,
  serialized_end=148,
)

_WRENCHSTAMPED.fields_by_name['time'].message_type = time__pb2._TIME
_WRENCHSTAMPED.fields_by_name['wrench'].message_type = wrench__pb2._WRENCH
DESCRIPTOR.message_types_by_name['WrenchStamped'] = _WRENCHSTAMPED
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

WrenchStamped = _reflection.GeneratedProtocolMessageType('WrenchStamped', (_message.Message,), dict(
  DESCRIPTOR = _WRENCHSTAMPED,
  __module__ = 'wrench_stamped_pb2'
  # @@protoc_insertion_point(class_scope:gazebo.msgs.WrenchStamped)
  ))
_sym_db.RegisterMessage(WrenchStamped)


# @@protoc_insertion_point(module_scope)
