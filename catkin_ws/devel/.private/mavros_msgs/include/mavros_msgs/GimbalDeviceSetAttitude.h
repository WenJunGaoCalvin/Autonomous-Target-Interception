// Generated by gencpp from file mavros_msgs/GimbalDeviceSetAttitude.msg
// DO NOT EDIT!


#ifndef MAVROS_MSGS_MESSAGE_GIMBALDEVICESETATTITUDE_H
#define MAVROS_MSGS_MESSAGE_GIMBALDEVICESETATTITUDE_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/Quaternion.h>

namespace mavros_msgs
{
template <class ContainerAllocator>
struct GimbalDeviceSetAttitude_
{
  typedef GimbalDeviceSetAttitude_<ContainerAllocator> Type;

  GimbalDeviceSetAttitude_()
    : target_system(0)
    , target_component(0)
    , flags(0)
    , q()
    , angular_velocity_x(0.0)
    , angular_velocity_y(0.0)
    , angular_velocity_z(0.0)  {
    }
  GimbalDeviceSetAttitude_(const ContainerAllocator& _alloc)
    : target_system(0)
    , target_component(0)
    , flags(0)
    , q(_alloc)
    , angular_velocity_x(0.0)
    , angular_velocity_y(0.0)
    , angular_velocity_z(0.0)  {
  (void)_alloc;
    }



   typedef uint8_t _target_system_type;
  _target_system_type target_system;

   typedef uint8_t _target_component_type;
  _target_component_type target_component;

   typedef uint16_t _flags_type;
  _flags_type flags;

   typedef  ::geometry_msgs::Quaternion_<ContainerAllocator>  _q_type;
  _q_type q;

   typedef float _angular_velocity_x_type;
  _angular_velocity_x_type angular_velocity_x;

   typedef float _angular_velocity_y_type;
  _angular_velocity_y_type angular_velocity_y;

   typedef float _angular_velocity_z_type;
  _angular_velocity_z_type angular_velocity_z;



// reducing the odds to have name collisions with Windows.h 
#if defined(_WIN32) && defined(GIMBAL_DEVICE_FLAGS_RETRACT)
  #undef GIMBAL_DEVICE_FLAGS_RETRACT
#endif
#if defined(_WIN32) && defined(GIMBAL_DEVICE_FLAGS_NEUTRAL)
  #undef GIMBAL_DEVICE_FLAGS_NEUTRAL
#endif
#if defined(_WIN32) && defined(GIMBAL_DEVICE_FLAGS_ROLL_LOCK)
  #undef GIMBAL_DEVICE_FLAGS_ROLL_LOCK
#endif
#if defined(_WIN32) && defined(GIMBAL_DEVICE_FLAGS_PITCH_LOCK)
  #undef GIMBAL_DEVICE_FLAGS_PITCH_LOCK
#endif
#if defined(_WIN32) && defined(GIMBAL_DEVICE_FLAGS_YAW_LOCK)
  #undef GIMBAL_DEVICE_FLAGS_YAW_LOCK
#endif

  enum {
    GIMBAL_DEVICE_FLAGS_RETRACT = 1u,
    GIMBAL_DEVICE_FLAGS_NEUTRAL = 2u,
    GIMBAL_DEVICE_FLAGS_ROLL_LOCK = 4u,
    GIMBAL_DEVICE_FLAGS_PITCH_LOCK = 8u,
    GIMBAL_DEVICE_FLAGS_YAW_LOCK = 16u,
  };


  typedef boost::shared_ptr< ::mavros_msgs::GimbalDeviceSetAttitude_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::mavros_msgs::GimbalDeviceSetAttitude_<ContainerAllocator> const> ConstPtr;

}; // struct GimbalDeviceSetAttitude_

typedef ::mavros_msgs::GimbalDeviceSetAttitude_<std::allocator<void> > GimbalDeviceSetAttitude;

typedef boost::shared_ptr< ::mavros_msgs::GimbalDeviceSetAttitude > GimbalDeviceSetAttitudePtr;
typedef boost::shared_ptr< ::mavros_msgs::GimbalDeviceSetAttitude const> GimbalDeviceSetAttitudeConstPtr;

// constants requiring out of line definition

   

   

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::mavros_msgs::GimbalDeviceSetAttitude_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::mavros_msgs::GimbalDeviceSetAttitude_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::mavros_msgs::GimbalDeviceSetAttitude_<ContainerAllocator1> & lhs, const ::mavros_msgs::GimbalDeviceSetAttitude_<ContainerAllocator2> & rhs)
{
  return lhs.target_system == rhs.target_system &&
    lhs.target_component == rhs.target_component &&
    lhs.flags == rhs.flags &&
    lhs.q == rhs.q &&
    lhs.angular_velocity_x == rhs.angular_velocity_x &&
    lhs.angular_velocity_y == rhs.angular_velocity_y &&
    lhs.angular_velocity_z == rhs.angular_velocity_z;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::mavros_msgs::GimbalDeviceSetAttitude_<ContainerAllocator1> & lhs, const ::mavros_msgs::GimbalDeviceSetAttitude_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace mavros_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::mavros_msgs::GimbalDeviceSetAttitude_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mavros_msgs::GimbalDeviceSetAttitude_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::mavros_msgs::GimbalDeviceSetAttitude_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::mavros_msgs::GimbalDeviceSetAttitude_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mavros_msgs::GimbalDeviceSetAttitude_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mavros_msgs::GimbalDeviceSetAttitude_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::mavros_msgs::GimbalDeviceSetAttitude_<ContainerAllocator> >
{
  static const char* value()
  {
    return "87a55ddad0e67129d73063fdc10f2ffb";
  }

  static const char* value(const ::mavros_msgs::GimbalDeviceSetAttitude_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x87a55ddad0e67129ULL;
  static const uint64_t static_value2 = 0xd73063fdc10f2ffbULL;
};

template<class ContainerAllocator>
struct DataType< ::mavros_msgs::GimbalDeviceSetAttitude_<ContainerAllocator> >
{
  static const char* value()
  {
    return "mavros_msgs/GimbalDeviceSetAttitude";
  }

  static const char* value(const ::mavros_msgs::GimbalDeviceSetAttitude_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::mavros_msgs::GimbalDeviceSetAttitude_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# MAVLink message: GIMBAL_DEVICE_SET_ATTITUDE\n"
"# https://mavlink.io/en/messages/common.html#GIMBAL_DEVICE_SET_ATTITUDE\n"
"\n"
"uint8 target_system         # System ID\n"
"uint8 target_component      # Component ID\n"
"\n"
"uint16 flags                # Low level gimbal flags (bitwise) - See GIMBAL_DEVICE_FLAGS\n"
"#GIMBAL_DEVICE_FLAGS\n"
"uint16 GIMBAL_DEVICE_FLAGS_RETRACT = 1     # Based on GIMBAL_DEVICE_FLAGS_RETRACT\n"
"uint16 GIMBAL_DEVICE_FLAGS_NEUTRAL = 2     # Based on GIMBAL_DEVICE_FLAGS_NEUTRAL\n"
"uint16 GIMBAL_DEVICE_FLAGS_ROLL_LOCK = 4   # Based on GIMBAL_DEVICE_FLAGS_ROLL_LOCK\n"
"uint16 GIMBAL_DEVICE_FLAGS_PITCH_LOCK = 8  # Based on GIMBAL_DEVICE_FLAGS_PITCH_LOCK\n"
"uint16 GIMBAL_DEVICE_FLAGS_YAW_LOCK = 16   # Based on GIMBAL_DEVICE_FLAGS_YAW_LOCK\n"
"\n"
"geometry_msgs/Quaternion q # Quaternion, x, y, z, w (0 0 0 1 is the null-rotation, the frame is depends on whether the flag GIMBAL_DEVICE_FLAGS_YAW_LOCK is set)\n"
"float32 angular_velocity_x # X component of angular velocity, positive is rolling to the right, NaN to be ignored.\n"
"float32 angular_velocity_y # Y component of angular velocity, positive is pitching up, NaN to be ignored.\n"
"float32 angular_velocity_z # Z component of angular velocity, positive is yawing to the right, NaN to be ignored.\n"
"================================================================================\n"
"MSG: geometry_msgs/Quaternion\n"
"# This represents an orientation in free space in quaternion form.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"float64 w\n"
;
  }

  static const char* value(const ::mavros_msgs::GimbalDeviceSetAttitude_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::mavros_msgs::GimbalDeviceSetAttitude_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.target_system);
      stream.next(m.target_component);
      stream.next(m.flags);
      stream.next(m.q);
      stream.next(m.angular_velocity_x);
      stream.next(m.angular_velocity_y);
      stream.next(m.angular_velocity_z);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GimbalDeviceSetAttitude_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::mavros_msgs::GimbalDeviceSetAttitude_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::mavros_msgs::GimbalDeviceSetAttitude_<ContainerAllocator>& v)
  {
    s << indent << "target_system: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.target_system);
    s << indent << "target_component: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.target_component);
    s << indent << "flags: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.flags);
    s << indent << "q: ";
    s << std::endl;
    Printer< ::geometry_msgs::Quaternion_<ContainerAllocator> >::stream(s, indent + "  ", v.q);
    s << indent << "angular_velocity_x: ";
    Printer<float>::stream(s, indent + "  ", v.angular_velocity_x);
    s << indent << "angular_velocity_y: ";
    Printer<float>::stream(s, indent + "  ", v.angular_velocity_y);
    s << indent << "angular_velocity_z: ";
    Printer<float>::stream(s, indent + "  ", v.angular_velocity_z);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MAVROS_MSGS_MESSAGE_GIMBALDEVICESETATTITUDE_H