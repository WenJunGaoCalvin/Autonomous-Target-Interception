// Generated by gencpp from file mavros_msgs/GimbalDeviceInformation.msg
// DO NOT EDIT!


#ifndef MAVROS_MSGS_MESSAGE_GIMBALDEVICEINFORMATION_H
#define MAVROS_MSGS_MESSAGE_GIMBALDEVICEINFORMATION_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace mavros_msgs
{
template <class ContainerAllocator>
struct GimbalDeviceInformation_
{
  typedef GimbalDeviceInformation_<ContainerAllocator> Type;

  GimbalDeviceInformation_()
    : header()
    , vendor_name()
    , model_name()
    , custom_name()
    , firmware_version(0)
    , hardware_version(0)
    , uid(0)
    , cap_flags(0)
    , custom_cap_flags(0)
    , roll_min(0.0)
    , roll_max(0.0)
    , pitch_min(0.0)
    , pitch_max(0.0)
    , yaw_min(0.0)
    , yaw_max(0.0)  {
    }
  GimbalDeviceInformation_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , vendor_name(_alloc)
    , model_name(_alloc)
    , custom_name(_alloc)
    , firmware_version(0)
    , hardware_version(0)
    , uid(0)
    , cap_flags(0)
    , custom_cap_flags(0)
    , roll_min(0.0)
    , roll_max(0.0)
    , pitch_min(0.0)
    , pitch_max(0.0)
    , yaw_min(0.0)
    , yaw_max(0.0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _vendor_name_type;
  _vendor_name_type vendor_name;

   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _model_name_type;
  _model_name_type model_name;

   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _custom_name_type;
  _custom_name_type custom_name;

   typedef uint32_t _firmware_version_type;
  _firmware_version_type firmware_version;

   typedef uint32_t _hardware_version_type;
  _hardware_version_type hardware_version;

   typedef uint64_t _uid_type;
  _uid_type uid;

   typedef uint32_t _cap_flags_type;
  _cap_flags_type cap_flags;

   typedef uint16_t _custom_cap_flags_type;
  _custom_cap_flags_type custom_cap_flags;

   typedef float _roll_min_type;
  _roll_min_type roll_min;

   typedef float _roll_max_type;
  _roll_max_type roll_max;

   typedef float _pitch_min_type;
  _pitch_min_type pitch_min;

   typedef float _pitch_max_type;
  _pitch_max_type pitch_max;

   typedef float _yaw_min_type;
  _yaw_min_type yaw_min;

   typedef float _yaw_max_type;
  _yaw_max_type yaw_max;



// reducing the odds to have name collisions with Windows.h 
#if defined(_WIN32) && defined(GIMBAL_DEVICE_CAP_FLAGS_HAS_RETRACT)
  #undef GIMBAL_DEVICE_CAP_FLAGS_HAS_RETRACT
#endif
#if defined(_WIN32) && defined(GIMBAL_DEVICE_CAP_FLAGS_HAS_NEUTRAL)
  #undef GIMBAL_DEVICE_CAP_FLAGS_HAS_NEUTRAL
#endif
#if defined(_WIN32) && defined(GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_AXIS)
  #undef GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_AXIS
#endif
#if defined(_WIN32) && defined(GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_FOLLOW)
  #undef GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_FOLLOW
#endif
#if defined(_WIN32) && defined(GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_LOCK)
  #undef GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_LOCK
#endif
#if defined(_WIN32) && defined(GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_AXIS)
  #undef GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_AXIS
#endif
#if defined(_WIN32) && defined(GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_FOLLOW)
  #undef GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_FOLLOW
#endif
#if defined(_WIN32) && defined(GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_LOCK)
  #undef GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_LOCK
#endif
#if defined(_WIN32) && defined(GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_AXIS)
  #undef GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_AXIS
#endif
#if defined(_WIN32) && defined(GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_FOLLOW)
  #undef GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_FOLLOW
#endif
#if defined(_WIN32) && defined(GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_LOCK)
  #undef GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_LOCK
#endif
#if defined(_WIN32) && defined(GIMBAL_DEVICE_CAP_FLAGS_SUPPORTS_INFINITE_YAW)
  #undef GIMBAL_DEVICE_CAP_FLAGS_SUPPORTS_INFINITE_YAW
#endif

  enum {
    GIMBAL_DEVICE_CAP_FLAGS_HAS_RETRACT = 1u,
    GIMBAL_DEVICE_CAP_FLAGS_HAS_NEUTRAL = 2u,
    GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_AXIS = 4u,
    GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_FOLLOW = 8u,
    GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_LOCK = 16u,
    GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_AXIS = 32u,
    GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_FOLLOW = 64u,
    GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_LOCK = 128u,
    GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_AXIS = 256u,
    GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_FOLLOW = 512u,
    GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_LOCK = 1024u,
    GIMBAL_DEVICE_CAP_FLAGS_SUPPORTS_INFINITE_YAW = 2048u,
  };


  typedef boost::shared_ptr< ::mavros_msgs::GimbalDeviceInformation_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::mavros_msgs::GimbalDeviceInformation_<ContainerAllocator> const> ConstPtr;

}; // struct GimbalDeviceInformation_

typedef ::mavros_msgs::GimbalDeviceInformation_<std::allocator<void> > GimbalDeviceInformation;

typedef boost::shared_ptr< ::mavros_msgs::GimbalDeviceInformation > GimbalDeviceInformationPtr;
typedef boost::shared_ptr< ::mavros_msgs::GimbalDeviceInformation const> GimbalDeviceInformationConstPtr;

// constants requiring out of line definition

   

   

   

   

   

   

   

   

   

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::mavros_msgs::GimbalDeviceInformation_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::mavros_msgs::GimbalDeviceInformation_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::mavros_msgs::GimbalDeviceInformation_<ContainerAllocator1> & lhs, const ::mavros_msgs::GimbalDeviceInformation_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.vendor_name == rhs.vendor_name &&
    lhs.model_name == rhs.model_name &&
    lhs.custom_name == rhs.custom_name &&
    lhs.firmware_version == rhs.firmware_version &&
    lhs.hardware_version == rhs.hardware_version &&
    lhs.uid == rhs.uid &&
    lhs.cap_flags == rhs.cap_flags &&
    lhs.custom_cap_flags == rhs.custom_cap_flags &&
    lhs.roll_min == rhs.roll_min &&
    lhs.roll_max == rhs.roll_max &&
    lhs.pitch_min == rhs.pitch_min &&
    lhs.pitch_max == rhs.pitch_max &&
    lhs.yaw_min == rhs.yaw_min &&
    lhs.yaw_max == rhs.yaw_max;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::mavros_msgs::GimbalDeviceInformation_<ContainerAllocator1> & lhs, const ::mavros_msgs::GimbalDeviceInformation_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace mavros_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::mavros_msgs::GimbalDeviceInformation_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mavros_msgs::GimbalDeviceInformation_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::mavros_msgs::GimbalDeviceInformation_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::mavros_msgs::GimbalDeviceInformation_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mavros_msgs::GimbalDeviceInformation_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mavros_msgs::GimbalDeviceInformation_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::mavros_msgs::GimbalDeviceInformation_<ContainerAllocator> >
{
  static const char* value()
  {
    return "cf9937985a9347aa15974b9c7eb2fa20";
  }

  static const char* value(const ::mavros_msgs::GimbalDeviceInformation_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xcf9937985a9347aaULL;
  static const uint64_t static_value2 = 0x15974b9c7eb2fa20ULL;
};

template<class ContainerAllocator>
struct DataType< ::mavros_msgs::GimbalDeviceInformation_<ContainerAllocator> >
{
  static const char* value()
  {
    return "mavros_msgs/GimbalDeviceInformation";
  }

  static const char* value(const ::mavros_msgs::GimbalDeviceInformation_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::mavros_msgs::GimbalDeviceInformation_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# MAVLink message: GIMBAL_DEVICE_INFORMATION\n"
"# https://mavlink.io/en/messages/common.html#GIMBAL_DEVICE_INFORMATION\n"
"\n"
"std_msgs/Header header\n"
"\n"
"string vendor_name # Name of the gimbal vendor.\n"
"string model_name # Name of the gimbal model.\n"
"string custom_name # Custom name of the gimbal given to it by the user.\n"
"uint32 firmware_version # Version of the gimbal firmware, encoded as: (Dev & 0xff) << 24 | (Patch & 0xff) << 16 | (Minor & 0xff) << 8 | (Major & 0xff).\n"
"uint32 hardware_version # Version of the gimbal hardware, encoded as: (Dev & 0xff) << 24 | (Patch & 0xff) << 16 | (Minor & 0xff) << 8 | (Major & 0xff).\n"
"uint64 uid # UID of gimbal hardware (0 if unknown).\n"
"\n"
"uint32 cap_flags # Bitmap of gimbal capability flags - see GIMBAL_DEVICE_CAP_FLAGS\n"
"#GIMBAL_DEVICE_CAP_FLAGS\n"
"uint32 GIMBAL_DEVICE_CAP_FLAGS_HAS_RETRACT = 1 # Gimbal device supports a retracted position\n"
"uint32 GIMBAL_DEVICE_CAP_FLAGS_HAS_NEUTRAL = 2 # Gimbal device supports a horizontal, forward looking position, stabilized\n"
"uint32 GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_AXIS = 4 # Gimbal device supports rotating around roll axis.\n"
"uint32 GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_FOLLOW = 8 # Gimbal device supports to follow a roll angle relative to the vehicle\n"
"uint32 GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_LOCK = 16 # Gimbal device supports locking to an roll angle (generally that's the default with roll stabilized)\n"
"uint32 GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_AXIS = 32 # Gimbal device supports rotating around pitch axis.\n"
"uint32 GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_FOLLOW = 64 # Gimbal device supports to follow a pitch angle relative to the vehicle\n"
"uint32 GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_LOCK = 128 # Gimbal device supports locking to an pitch angle (generally that's the default with pitch stabilized)\n"
"uint32 GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_AXIS = 256 # Gimbal device supports rotating around yaw axis.\n"
"uint32 GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_FOLLOW = 512 # Gimbal device supports to follow a yaw angle relative to the vehicle (generally that's the default)\n"
"uint32 GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_LOCK = 1024 # Gimbal device supports locking to an absolute heading (often this is an option available)\n"
"uint32 GIMBAL_DEVICE_CAP_FLAGS_SUPPORTS_INFINITE_YAW = 2048 # Gimbal device supports yawing/panning infinetely (e.g. using slip disk).\n"
"\n"
"uint16 custom_cap_flags # Bitmap for use for gimbal-specific capability flags.\n"
"float32 roll_min # Minimum hardware roll angle (positive: rolling to the right, negative: rolling to the left)\n"
"float32 roll_max # Maximum hardware roll angle (positive: rolling to the right, negative: rolling to the left)\n"
"float32 pitch_min # Minimum pitch angle (positive: up, negative: down)\n"
"float32 pitch_max # Maximum pitch angle (positive: up, negative: down)\n"
"float32 yaw_min # Minimum yaw angle (positive: to the right, negative: to the left)\n"
"float32 yaw_max # Maximum yaw angle (positive: to the right, negative: to the left)\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
;
  }

  static const char* value(const ::mavros_msgs::GimbalDeviceInformation_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::mavros_msgs::GimbalDeviceInformation_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.vendor_name);
      stream.next(m.model_name);
      stream.next(m.custom_name);
      stream.next(m.firmware_version);
      stream.next(m.hardware_version);
      stream.next(m.uid);
      stream.next(m.cap_flags);
      stream.next(m.custom_cap_flags);
      stream.next(m.roll_min);
      stream.next(m.roll_max);
      stream.next(m.pitch_min);
      stream.next(m.pitch_max);
      stream.next(m.yaw_min);
      stream.next(m.yaw_max);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GimbalDeviceInformation_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::mavros_msgs::GimbalDeviceInformation_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::mavros_msgs::GimbalDeviceInformation_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "vendor_name: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.vendor_name);
    s << indent << "model_name: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.model_name);
    s << indent << "custom_name: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.custom_name);
    s << indent << "firmware_version: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.firmware_version);
    s << indent << "hardware_version: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.hardware_version);
    s << indent << "uid: ";
    Printer<uint64_t>::stream(s, indent + "  ", v.uid);
    s << indent << "cap_flags: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.cap_flags);
    s << indent << "custom_cap_flags: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.custom_cap_flags);
    s << indent << "roll_min: ";
    Printer<float>::stream(s, indent + "  ", v.roll_min);
    s << indent << "roll_max: ";
    Printer<float>::stream(s, indent + "  ", v.roll_max);
    s << indent << "pitch_min: ";
    Printer<float>::stream(s, indent + "  ", v.pitch_min);
    s << indent << "pitch_max: ";
    Printer<float>::stream(s, indent + "  ", v.pitch_max);
    s << indent << "yaw_min: ";
    Printer<float>::stream(s, indent + "  ", v.yaw_min);
    s << indent << "yaw_max: ";
    Printer<float>::stream(s, indent + "  ", v.yaw_max);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MAVROS_MSGS_MESSAGE_GIMBALDEVICEINFORMATION_H
