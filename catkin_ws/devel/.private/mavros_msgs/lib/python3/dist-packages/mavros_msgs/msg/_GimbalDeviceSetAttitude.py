# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from mavros_msgs/GimbalDeviceSetAttitude.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import geometry_msgs.msg

class GimbalDeviceSetAttitude(genpy.Message):
  _md5sum = "87a55ddad0e67129d73063fdc10f2ffb"
  _type = "mavros_msgs/GimbalDeviceSetAttitude"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """# MAVLink message: GIMBAL_DEVICE_SET_ATTITUDE
# https://mavlink.io/en/messages/common.html#GIMBAL_DEVICE_SET_ATTITUDE

uint8 target_system         # System ID
uint8 target_component      # Component ID

uint16 flags                # Low level gimbal flags (bitwise) - See GIMBAL_DEVICE_FLAGS
#GIMBAL_DEVICE_FLAGS
uint16 GIMBAL_DEVICE_FLAGS_RETRACT = 1     # Based on GIMBAL_DEVICE_FLAGS_RETRACT
uint16 GIMBAL_DEVICE_FLAGS_NEUTRAL = 2     # Based on GIMBAL_DEVICE_FLAGS_NEUTRAL
uint16 GIMBAL_DEVICE_FLAGS_ROLL_LOCK = 4   # Based on GIMBAL_DEVICE_FLAGS_ROLL_LOCK
uint16 GIMBAL_DEVICE_FLAGS_PITCH_LOCK = 8  # Based on GIMBAL_DEVICE_FLAGS_PITCH_LOCK
uint16 GIMBAL_DEVICE_FLAGS_YAW_LOCK = 16   # Based on GIMBAL_DEVICE_FLAGS_YAW_LOCK

geometry_msgs/Quaternion q # Quaternion, x, y, z, w (0 0 0 1 is the null-rotation, the frame is depends on whether the flag GIMBAL_DEVICE_FLAGS_YAW_LOCK is set)
float32 angular_velocity_x # X component of angular velocity, positive is rolling to the right, NaN to be ignored.
float32 angular_velocity_y # Y component of angular velocity, positive is pitching up, NaN to be ignored.
float32 angular_velocity_z # Z component of angular velocity, positive is yawing to the right, NaN to be ignored.
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
"""
  # Pseudo-constants
  GIMBAL_DEVICE_FLAGS_RETRACT = 1
  GIMBAL_DEVICE_FLAGS_NEUTRAL = 2
  GIMBAL_DEVICE_FLAGS_ROLL_LOCK = 4
  GIMBAL_DEVICE_FLAGS_PITCH_LOCK = 8
  GIMBAL_DEVICE_FLAGS_YAW_LOCK = 16

  __slots__ = ['target_system','target_component','flags','q','angular_velocity_x','angular_velocity_y','angular_velocity_z']
  _slot_types = ['uint8','uint8','uint16','geometry_msgs/Quaternion','float32','float32','float32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       target_system,target_component,flags,q,angular_velocity_x,angular_velocity_y,angular_velocity_z

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(GimbalDeviceSetAttitude, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.target_system is None:
        self.target_system = 0
      if self.target_component is None:
        self.target_component = 0
      if self.flags is None:
        self.flags = 0
      if self.q is None:
        self.q = geometry_msgs.msg.Quaternion()
      if self.angular_velocity_x is None:
        self.angular_velocity_x = 0.
      if self.angular_velocity_y is None:
        self.angular_velocity_y = 0.
      if self.angular_velocity_z is None:
        self.angular_velocity_z = 0.
    else:
      self.target_system = 0
      self.target_component = 0
      self.flags = 0
      self.q = geometry_msgs.msg.Quaternion()
      self.angular_velocity_x = 0.
      self.angular_velocity_y = 0.
      self.angular_velocity_z = 0.

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_get_struct_2BH4d3f().pack(_x.target_system, _x.target_component, _x.flags, _x.q.x, _x.q.y, _x.q.z, _x.q.w, _x.angular_velocity_x, _x.angular_velocity_y, _x.angular_velocity_z))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.q is None:
        self.q = geometry_msgs.msg.Quaternion()
      end = 0
      _x = self
      start = end
      end += 48
      (_x.target_system, _x.target_component, _x.flags, _x.q.x, _x.q.y, _x.q.z, _x.q.w, _x.angular_velocity_x, _x.angular_velocity_y, _x.angular_velocity_z,) = _get_struct_2BH4d3f().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_2BH4d3f().pack(_x.target_system, _x.target_component, _x.flags, _x.q.x, _x.q.y, _x.q.z, _x.q.w, _x.angular_velocity_x, _x.angular_velocity_y, _x.angular_velocity_z))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.q is None:
        self.q = geometry_msgs.msg.Quaternion()
      end = 0
      _x = self
      start = end
      end += 48
      (_x.target_system, _x.target_component, _x.flags, _x.q.x, _x.q.y, _x.q.z, _x.q.w, _x.angular_velocity_x, _x.angular_velocity_y, _x.angular_velocity_z,) = _get_struct_2BH4d3f().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_2BH4d3f = None
def _get_struct_2BH4d3f():
    global _struct_2BH4d3f
    if _struct_2BH4d3f is None:
        _struct_2BH4d3f = struct.Struct("<2BH4d3f")
    return _struct_2BH4d3f