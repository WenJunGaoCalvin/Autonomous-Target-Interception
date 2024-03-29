// Generated by gencpp from file mavros_msgs/GimbalManagerInformation.msg
// DO NOT EDIT!


#ifndef MAVROS_MSGS_MESSAGE_GIMBALMANAGERINFORMATION_H
#define MAVROS_MSGS_MESSAGE_GIMBALMANAGERINFORMATION_H


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
struct GimbalManagerInformation_
{
  typedef GimbalManagerInformation_<ContainerAllocator> Type;

  GimbalManagerInformation_()
    : header()
    , cap_flags(0)
    , gimbal_device_id(0)
    , roll_min(0.0)
    , roll_max(0.0)
    , pitch_min(0.0)
    , pitch_max(0.0)
    , yaw_min(0.0)
    , yaw_max(0.0)  {
    }
  GimbalManagerInformation_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , cap_flags(0)
    , gimbal_device_id(0)
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

   typedef uint32_t _cap_flags_type;
  _cap_flags_type cap_flags;

   typedef uint8_t _gimbal_device_id_type;
  _gimbal_device_id_type gimbal_device_id;

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
#if defined(_WIN32) && defined(GIMBAL_MANAGER_CAP_FLAGS_HAS_RETRACT)
  #undef GIMBAL_MANAGER_CAP_FLAGS_HAS_RETRACT
#endif
#if defined(_WIN32) && defined(GIMBAL_MANAGER_CAP_FLAGS_HAS_NEUTRAL)
  #undef GIMBAL_MANAGER_CAP_FLAGS_HAS_NEUTRAL
#endif
#if defined(_WIN32) && defined(GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_AXIS)
  #undef GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_AXIS
#endif
#if defined(_WIN32) && defined(GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_FOLLOW)
  #undef GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_FOLLOW
#endif
#if defined(_WIN32) && defined(GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_LOCK)
  #undef GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_LOCK
#endif
#if defined(_WIN32) && defined(GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_AXIS)
  #undef GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_AXIS
#endif
#if defined(_WIN32) && defined(GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_FOLLOW)
  #undef GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_FOLLOW
#endif
#if defined(_WIN32) && defined(GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_LOCK)
  #undef GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_LOCK
#endif
#if defined(_WIN32) && defined(GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_AXIS)
  #undef GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_AXIS
#endif
#if defined(_WIN32) && defined(GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_FOLLOW)
  #undef GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_FOLLOW
#endif
#if defined(_WIN32) && defined(GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_LOCK)
  #undef GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_LOCK
#endif
#if defined(_WIN32) && defined(GIMBAL_MANAGER_CAP_FLAGS_SUPPORTS_INFINITE_YAW)
  #undef GIMBAL_MANAGER_CAP_FLAGS_SUPPORTS_INFINITE_YAW
#endif
#if defined(_WIN32) && defined(GIMBAL_MANAGER_CAP_FLAGS_CAN_POINT_LOCATION_LOCAL)
  #undef GIMBAL_MANAGER_CAP_FLAGS_CAN_POINT_LOCATION_LOCAL
#endif
#if defined(_WIN32) && defined(GIMBAL_MANAGER_CAP_FLAGS_CAN_POINT_LOCATION_GLOBAL)
  #undef GIMBAL_MANAGER_CAP_FLAGS_CAN_POINT_LOCATION_GLOBAL
#endif

  enum {
    GIMBAL_MANAGER_CAP_FLAGS_HAS_RETRACT = 1u,
    GIMBAL_MANAGER_CAP_FLAGS_HAS_NEUTRAL = 2u,
    GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_AXIS = 4u,
    GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_FOLLOW = 8u,
    GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_LOCK = 16u,
    GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_AXIS = 32u,
    GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_FOLLOW = 64u,
    GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_LOCK = 128u,
    GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_AXIS = 256u,
    GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_FOLLOW = 512u,
    GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_LOCK = 1024u,
    GIMBAL_MANAGER_CAP_FLAGS_SUPPORTS_INFINITE_YAW = 2048u,
    GIMBAL_MANAGER_CAP_FLAGS_CAN_POINT_LOCATION_LOCAL = 65536u,
    GIMBAL_MANAGER_CAP_FLAGS_CAN_POINT_LOCATION_GLOBAL = 131072u,
  };


  typedef boost::shared_ptr< ::mavros_msgs::GimbalManagerInformation_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::mavros_msgs::GimbalManagerInformation_<ContainerAllocator> const> ConstPtr;

}; // struct GimbalManagerInformation_

typedef ::mavros_msgs::GimbalManagerInformation_<std::allocator<void> > GimbalManagerInformation;

typedef boost::shared_ptr< ::mavros_msgs::GimbalManagerInformation > GimbalManagerInformationPtr;
typedef boost::shared_ptr< ::mavros_msgs::GimbalManagerInformation const> GimbalManagerInformationConstPtr;

// constants requiring out of line definition

   

   

   

   

   

   

   

   

   

   

   

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::mavros_msgs::GimbalManagerInformation_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::mavros_msgs::GimbalManagerInformation_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::mavros_msgs::GimbalManagerInformation_<ContainerAllocator1> & lhs, const ::mavros_msgs::GimbalManagerInformation_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.cap_flags == rhs.cap_flags &&
    lhs.gimbal_device_id == rhs.gimbal_device_id &&
    lhs.roll_min == rhs.roll_min &&
    lhs.roll_max == rhs.roll_max &&
    lhs.pitch_min == rhs.pitch_min &&
    lhs.pitch_max == rhs.pitch_max &&
    lhs.yaw_min == rhs.yaw_min &&
    lhs.yaw_max == rhs.yaw_max;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::mavros_msgs::GimbalManagerInformation_<ContainerAllocator1> & lhs, const ::mavros_msgs::GimbalManagerInformation_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace mavros_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::mavros_msgs::GimbalManagerInformation_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mavros_msgs::GimbalManagerInformation_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::mavros_msgs::GimbalManagerInformation_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::mavros_msgs::GimbalManagerInformation_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mavros_msgs::GimbalManagerInformation_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mavros_msgs::GimbalManagerInformation_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::mavros_msgs::GimbalManagerInformation_<ContainerAllocator> >
{
  static const char* value()
  {
    return "afbf4fbefec70c70e001795d309516f7";
  }

  static const char* value(const ::mavros_msgs::GimbalManagerInformation_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xafbf4fbefec70c70ULL;
  static const uint64_t static_value2 = 0xe001795d309516f7ULL;
};

template<class ContainerAllocator>
struct DataType< ::mavros_msgs::GimbalManagerInformation_<ContainerAllocator> >
{
  static const char* value()
  {
    return "mavros_msgs/GimbalManagerInformation";
  }

  static const char* value(const ::mavros_msgs::GimbalManagerInformation_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::mavros_msgs::GimbalManagerInformation_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# MAVLink message: GIMBAL_MANAGER_INFORMATION\n"
"# https://mavlink.io/en/messages/common.html#GIMBAL_MANAGER_INFORMATION\n"
"\n"
"std_msgs/Header header\n"
"\n"
"uint32 cap_flags # Bitmap of gimbal capability flags - see GIMBAL_MANAGER_CAP_FLAGS\n"
"#GIMBAL_MANAGER_CAP_FLAGS\n"
"uint32 GIMBAL_MANAGER_CAP_FLAGS_HAS_RETRACT = 1 # Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_RETRACT.\n"
"uint32 GIMBAL_MANAGER_CAP_FLAGS_HAS_NEUTRAL = 2 # Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_NEUTRAL.\n"
"uint32 GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_AXIS = 4 # Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_AXIS.\n"
"uint32 GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_FOLLOW = 8 # Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_FOLLOW.\n"
"uint32 GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_LOCK = 16 # Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_LOCK.\n"
"uint32 GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_AXIS = 32 # Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_AXIS.\n"
"uint32 GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_FOLLOW = 64 # Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_FOLLOW.\n"
"uint32 GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_LOCK = 128 # Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_LOCK.\n"
"uint32 GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_AXIS = 256 # Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_AXIS.\n"
"uint32 GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_FOLLOW = 512 # Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_FOLLOW.\n"
"uint32 GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_LOCK = 1024 # Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_LOCK.\n"
"uint32 GIMBAL_MANAGER_CAP_FLAGS_SUPPORTS_INFINITE_YAW = 2048 # Based on GIMBAL_DEVICE_CAP_FLAGS_SUPPORTS_INFINITE_YAW.\n"
"uint32 GIMBAL_MANAGER_CAP_FLAGS_CAN_POINT_LOCATION_LOCAL = 65536 # Gimbal manager supports to point to a local position.\n"
"uint32 GIMBAL_MANAGER_CAP_FLAGS_CAN_POINT_LOCATION_GLOBAL = 131072 # Gimbal manager supports to point to a global latitude, longitude, altitude position.\n"
"\n"
"uint8 gimbal_device_id # Gimbal device ID that this gimbal manager is responsible for.\n"
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

  static const char* value(const ::mavros_msgs::GimbalManagerInformation_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::mavros_msgs::GimbalManagerInformation_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.cap_flags);
      stream.next(m.gimbal_device_id);
      stream.next(m.roll_min);
      stream.next(m.roll_max);
      stream.next(m.pitch_min);
      stream.next(m.pitch_max);
      stream.next(m.yaw_min);
      stream.next(m.yaw_max);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GimbalManagerInformation_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::mavros_msgs::GimbalManagerInformation_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::mavros_msgs::GimbalManagerInformation_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "cap_flags: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.cap_flags);
    s << indent << "gimbal_device_id: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.gimbal_device_id);
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

#endif // MAVROS_MSGS_MESSAGE_GIMBALMANAGERINFORMATION_H
