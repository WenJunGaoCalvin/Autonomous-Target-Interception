# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: cessna.proto

import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='cessna.proto',
  package='gazebo.msgs',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=_b('\n\x0c\x63\x65ssna.proto\x12\x0bgazebo.msgs\"\xc4\x02\n\x06\x43\x65ssna\x12\x17\n\x0fpropeller_speed\x18\x01 \x01(\x02\x12\x14\n\x0cleft_aileron\x18\x02 \x01(\x02\x12\x11\n\tleft_flap\x18\x03 \x01(\x02\x12\x15\n\rright_aileron\x18\x04 \x01(\x02\x12\x12\n\nright_flap\x18\x05 \x01(\x02\x12\x11\n\televators\x18\x06 \x01(\x02\x12\x0e\n\x06rudder\x18\x07 \x01(\x02\x12\x1b\n\x13\x63md_propeller_speed\x18\x08 \x01(\x02\x12\x18\n\x10\x63md_left_aileron\x18\t \x01(\x02\x12\x15\n\rcmd_left_flap\x18\n \x01(\x02\x12\x19\n\x11\x63md_right_aileron\x18\x0b \x01(\x02\x12\x16\n\x0e\x63md_right_flap\x18\x0c \x01(\x02\x12\x15\n\rcmd_elevators\x18\r \x01(\x02\x12\x12\n\ncmd_rudder\x18\x0e \x01(\x02')
)




_CESSNA = _descriptor.Descriptor(
  name='Cessna',
  full_name='gazebo.msgs.Cessna',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='propeller_speed', full_name='gazebo.msgs.Cessna.propeller_speed', index=0,
      number=1, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='left_aileron', full_name='gazebo.msgs.Cessna.left_aileron', index=1,
      number=2, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='left_flap', full_name='gazebo.msgs.Cessna.left_flap', index=2,
      number=3, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='right_aileron', full_name='gazebo.msgs.Cessna.right_aileron', index=3,
      number=4, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='right_flap', full_name='gazebo.msgs.Cessna.right_flap', index=4,
      number=5, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='elevators', full_name='gazebo.msgs.Cessna.elevators', index=5,
      number=6, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='rudder', full_name='gazebo.msgs.Cessna.rudder', index=6,
      number=7, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='cmd_propeller_speed', full_name='gazebo.msgs.Cessna.cmd_propeller_speed', index=7,
      number=8, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='cmd_left_aileron', full_name='gazebo.msgs.Cessna.cmd_left_aileron', index=8,
      number=9, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='cmd_left_flap', full_name='gazebo.msgs.Cessna.cmd_left_flap', index=9,
      number=10, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='cmd_right_aileron', full_name='gazebo.msgs.Cessna.cmd_right_aileron', index=10,
      number=11, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='cmd_right_flap', full_name='gazebo.msgs.Cessna.cmd_right_flap', index=11,
      number=12, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='cmd_elevators', full_name='gazebo.msgs.Cessna.cmd_elevators', index=12,
      number=13, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='cmd_rudder', full_name='gazebo.msgs.Cessna.cmd_rudder', index=13,
      number=14, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
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
  serialized_start=30,
  serialized_end=354,
)

DESCRIPTOR.message_types_by_name['Cessna'] = _CESSNA
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

Cessna = _reflection.GeneratedProtocolMessageType('Cessna', (_message.Message,), dict(
  DESCRIPTOR = _CESSNA,
  __module__ = 'cessna_pb2'
  # @@protoc_insertion_point(class_scope:gazebo.msgs.Cessna)
  ))
_sym_db.RegisterMessage(Cessna)


# @@protoc_insertion_point(module_scope)
