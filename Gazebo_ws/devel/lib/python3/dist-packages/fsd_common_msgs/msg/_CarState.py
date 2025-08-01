# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from fsd_common_msgs/CarState.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import fsd_common_msgs.msg
import geometry_msgs.msg
import std_msgs.msg

class CarState(genpy.Message):
  _md5sum = "be929b6a6d1d1d2c80065a8e530992fd"
  _type = "fsd_common_msgs/CarState"
  _has_header = True  # flag to mark the presence of a Header object
  _full_text = """std_msgs/Header header

geometry_msgs/Pose2D car_state    			# Position in x, y, theta
fsd_common_msgs/CarStateDt car_state_dt		# Velocities
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: geometry_msgs/Pose2D
# Deprecated
# Please use the full 3D pose.

# In general our recommendation is to use a full 3D representation of everything and for 2D specific applications make the appropriate projections into the plane for their calculations but optimally will preserve the 3D information during processing.

# If we have parallel copies of 2D datatypes every UI and other pipeline will end up needing to have dual interfaces to plot everything. And you will end up with not being able to use 3D tools for 2D use cases even if they're completely valid, as you'd have to reimplement it with different inputs and outputs. It's not particularly hard to plot the 2D pose or compute the yaw error for the Pose message and there are already tools and libraries that can do this for you.


# This expresses a position and orientation on a 2D manifold.

float64 x
float64 y
float64 theta

================================================================================
MSG: fsd_common_msgs/CarStateDt
std_msgs/Header header

geometry_msgs/Pose2D car_state_dt   # Velocity in x, y, theta"""
  __slots__ = ['header','car_state','car_state_dt']
  _slot_types = ['std_msgs/Header','geometry_msgs/Pose2D','fsd_common_msgs/CarStateDt']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       header,car_state,car_state_dt

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(CarState, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.car_state is None:
        self.car_state = geometry_msgs.msg.Pose2D()
      if self.car_state_dt is None:
        self.car_state_dt = fsd_common_msgs.msg.CarStateDt()
    else:
      self.header = std_msgs.msg.Header()
      self.car_state = geometry_msgs.msg.Pose2D()
      self.car_state_dt = fsd_common_msgs.msg.CarStateDt()

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
      buff.write(_get_struct_3I().pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_3d3I().pack(_x.car_state.x, _x.car_state.y, _x.car_state.theta, _x.car_state_dt.header.seq, _x.car_state_dt.header.stamp.secs, _x.car_state_dt.header.stamp.nsecs))
      _x = self.car_state_dt.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_3d().pack(_x.car_state_dt.car_state_dt.x, _x.car_state_dt.car_state_dt.y, _x.car_state_dt.car_state_dt.theta))
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
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.car_state is None:
        self.car_state = geometry_msgs.msg.Pose2D()
      if self.car_state_dt is None:
        self.car_state_dt = fsd_common_msgs.msg.CarStateDt()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 36
      (_x.car_state.x, _x.car_state.y, _x.car_state.theta, _x.car_state_dt.header.seq, _x.car_state_dt.header.stamp.secs, _x.car_state_dt.header.stamp.nsecs,) = _get_struct_3d3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.car_state_dt.header.frame_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.car_state_dt.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 24
      (_x.car_state_dt.car_state_dt.x, _x.car_state_dt.car_state_dt.y, _x.car_state_dt.car_state_dt.theta,) = _get_struct_3d().unpack(str[start:end])
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
      buff.write(_get_struct_3I().pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_3d3I().pack(_x.car_state.x, _x.car_state.y, _x.car_state.theta, _x.car_state_dt.header.seq, _x.car_state_dt.header.stamp.secs, _x.car_state_dt.header.stamp.nsecs))
      _x = self.car_state_dt.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_3d().pack(_x.car_state_dt.car_state_dt.x, _x.car_state_dt.car_state_dt.y, _x.car_state_dt.car_state_dt.theta))
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
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.car_state is None:
        self.car_state = geometry_msgs.msg.Pose2D()
      if self.car_state_dt is None:
        self.car_state_dt = fsd_common_msgs.msg.CarStateDt()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 36
      (_x.car_state.x, _x.car_state.y, _x.car_state.theta, _x.car_state_dt.header.seq, _x.car_state_dt.header.stamp.secs, _x.car_state_dt.header.stamp.nsecs,) = _get_struct_3d3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.car_state_dt.header.frame_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.car_state_dt.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 24
      (_x.car_state_dt.car_state_dt.x, _x.car_state_dt.car_state_dt.y, _x.car_state_dt.car_state_dt.theta,) = _get_struct_3d().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_3I = None
def _get_struct_3I():
    global _struct_3I
    if _struct_3I is None:
        _struct_3I = struct.Struct("<3I")
    return _struct_3I
_struct_3d = None
def _get_struct_3d():
    global _struct_3d
    if _struct_3d is None:
        _struct_3d = struct.Struct("<3d")
    return _struct_3d
_struct_3d3I = None
def _get_struct_3d3I():
    global _struct_3d3I
    if _struct_3d3I is None:
        _struct_3d3I = struct.Struct("<3d3I")
    return _struct_3d3I
