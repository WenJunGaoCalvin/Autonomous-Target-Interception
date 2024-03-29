// Generated by gencpp from file mavros_msgs/GimbalManagerCameraTrackRequest.msg
// DO NOT EDIT!


#ifndef MAVROS_MSGS_MESSAGE_GIMBALMANAGERCAMERATRACKREQUEST_H
#define MAVROS_MSGS_MESSAGE_GIMBALMANAGERCAMERATRACKREQUEST_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace mavros_msgs
{
template <class ContainerAllocator>
struct GimbalManagerCameraTrackRequest_
{
  typedef GimbalManagerCameraTrackRequest_<ContainerAllocator> Type;

  GimbalManagerCameraTrackRequest_()
    : mode(0)
    , x(0.0)
    , y(0.0)
    , radius(0.0)
    , top_left_x(0.0)
    , top_left_y(0.0)
    , bottom_right_x(0.0)
    , bottom_right_y(0.0)  {
    }
  GimbalManagerCameraTrackRequest_(const ContainerAllocator& _alloc)
    : mode(0)
    , x(0.0)
    , y(0.0)
    , radius(0.0)
    , top_left_x(0.0)
    , top_left_y(0.0)
    , bottom_right_x(0.0)
    , bottom_right_y(0.0)  {
  (void)_alloc;
    }



   typedef uint8_t _mode_type;
  _mode_type mode;

   typedef float _x_type;
  _x_type x;

   typedef float _y_type;
  _y_type y;

   typedef float _radius_type;
  _radius_type radius;

   typedef float _top_left_x_type;
  _top_left_x_type top_left_x;

   typedef float _top_left_y_type;
  _top_left_y_type top_left_y;

   typedef float _bottom_right_x_type;
  _bottom_right_x_type bottom_right_x;

   typedef float _bottom_right_y_type;
  _bottom_right_y_type bottom_right_y;



// reducing the odds to have name collisions with Windows.h 
#if defined(_WIN32) && defined(CAMERA_TRACK_MODE_POINT)
  #undef CAMERA_TRACK_MODE_POINT
#endif
#if defined(_WIN32) && defined(CAMERA_TRACK_MODE_RECTANGLE)
  #undef CAMERA_TRACK_MODE_RECTANGLE
#endif
#if defined(_WIN32) && defined(CAMERA_TRACK_MODE_STOP_TRACKING)
  #undef CAMERA_TRACK_MODE_STOP_TRACKING
#endif

  enum {
    CAMERA_TRACK_MODE_POINT = 0u,
    CAMERA_TRACK_MODE_RECTANGLE = 1u,
    CAMERA_TRACK_MODE_STOP_TRACKING = 2u,
  };


  typedef boost::shared_ptr< ::mavros_msgs::GimbalManagerCameraTrackRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::mavros_msgs::GimbalManagerCameraTrackRequest_<ContainerAllocator> const> ConstPtr;

}; // struct GimbalManagerCameraTrackRequest_

typedef ::mavros_msgs::GimbalManagerCameraTrackRequest_<std::allocator<void> > GimbalManagerCameraTrackRequest;

typedef boost::shared_ptr< ::mavros_msgs::GimbalManagerCameraTrackRequest > GimbalManagerCameraTrackRequestPtr;
typedef boost::shared_ptr< ::mavros_msgs::GimbalManagerCameraTrackRequest const> GimbalManagerCameraTrackRequestConstPtr;

// constants requiring out of line definition

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::mavros_msgs::GimbalManagerCameraTrackRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::mavros_msgs::GimbalManagerCameraTrackRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::mavros_msgs::GimbalManagerCameraTrackRequest_<ContainerAllocator1> & lhs, const ::mavros_msgs::GimbalManagerCameraTrackRequest_<ContainerAllocator2> & rhs)
{
  return lhs.mode == rhs.mode &&
    lhs.x == rhs.x &&
    lhs.y == rhs.y &&
    lhs.radius == rhs.radius &&
    lhs.top_left_x == rhs.top_left_x &&
    lhs.top_left_y == rhs.top_left_y &&
    lhs.bottom_right_x == rhs.bottom_right_x &&
    lhs.bottom_right_y == rhs.bottom_right_y;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::mavros_msgs::GimbalManagerCameraTrackRequest_<ContainerAllocator1> & lhs, const ::mavros_msgs::GimbalManagerCameraTrackRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace mavros_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::mavros_msgs::GimbalManagerCameraTrackRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mavros_msgs::GimbalManagerCameraTrackRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::mavros_msgs::GimbalManagerCameraTrackRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::mavros_msgs::GimbalManagerCameraTrackRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mavros_msgs::GimbalManagerCameraTrackRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mavros_msgs::GimbalManagerCameraTrackRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::mavros_msgs::GimbalManagerCameraTrackRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ba0d64325a3002df0189bd5ebdeecac8";
  }

  static const char* value(const ::mavros_msgs::GimbalManagerCameraTrackRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xba0d64325a3002dfULL;
  static const uint64_t static_value2 = 0x0189bd5ebdeecac8ULL;
};

template<class ContainerAllocator>
struct DataType< ::mavros_msgs::GimbalManagerCameraTrackRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "mavros_msgs/GimbalManagerCameraTrackRequest";
  }

  static const char* value(const ::mavros_msgs::GimbalManagerCameraTrackRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::mavros_msgs::GimbalManagerCameraTrackRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# MAVLink commands: CAMERA_TRACK_POINT, CAMERA_TRACK_RECTANGLE, CAMERA_STOP_TRACKING\n"
"# https://mavlink.io/en/messages/common.html#MAV_CMD_CAMERA_TRACK_POINT\n"
"# https://mavlink.io/en/messages/common.html#MAV_CMD_CAMERA_TRACK_RECTANGLE\n"
"# https://mavlink.io/en/messages/common.html#MAV_CMD_CAMERA_STOP_TRACKING\n"
"\n"
"uint8 mode      # enumerator to indicate camera track mode setting - see CAMERA_TRACK_MODE\n"
"#CAMERA_TRACK_MODE\n"
"uint8 CAMERA_TRACK_MODE_POINT = 0           # If the camera supports point visual tracking (CAMERA_CAP_FLAGS_HAS_TRACKING_POINT is set), this command allows to initiate the tracking. [CAMERA_TRACK_POINT]\n"
"uint8 CAMERA_TRACK_MODE_RECTANGLE = 1       # If the camera supports rectangle visual tracking (CAMERA_CAP_FLAGS_HAS_TRACKING_RECTANGLE is set), this command allows to initiate the tracking. [CAMERA_TRACK_RECTANGLE]\n"
"uint8 CAMERA_TRACK_MODE_STOP_TRACKING = 2   # Stops ongoing tracking. [CAMERA_STOP_TRACKING]\n"
"\n"
"#For CAMERA_TRACK_POINT\n"
"float32 x       # Point to track x value (normalized 0..1, 0 is left, 1 is right).\n"
"float32 y       # Point to track y value (normalized 0..1, 0 is top, 1 is bottom).\n"
"float32 radius  # Point radius (normalized 0..1, 0 is image left, 1 is image right).\n"
"\n"
"#For CAMERA_TRACK_RECTANGLE\n"
"float32 top_left_x      # Top left corner of rectangle x value (normalized 0..1, 0 is left, 1 is right).\n"
"float32 top_left_y      # Top left corner of rectangle y value (normalized 0..1, 0 is top, 1 is bottom).\n"
"float32 bottom_right_x  # Bottom right corner of rectangle x value (normalized 0..1, 0 is left, 1 is right).\n"
"float32 bottom_right_y  # Bottom right corner of rectangle y value (normalized 0..1, 0 is top, 1 is bottom).\n"
"\n"
"#CAMERA_STOP_TRACKING doesn't take extra parameters\n"
"\n"
;
  }

  static const char* value(const ::mavros_msgs::GimbalManagerCameraTrackRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::mavros_msgs::GimbalManagerCameraTrackRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.mode);
      stream.next(m.x);
      stream.next(m.y);
      stream.next(m.radius);
      stream.next(m.top_left_x);
      stream.next(m.top_left_y);
      stream.next(m.bottom_right_x);
      stream.next(m.bottom_right_y);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GimbalManagerCameraTrackRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::mavros_msgs::GimbalManagerCameraTrackRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::mavros_msgs::GimbalManagerCameraTrackRequest_<ContainerAllocator>& v)
  {
    s << indent << "mode: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.mode);
    s << indent << "x: ";
    Printer<float>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<float>::stream(s, indent + "  ", v.y);
    s << indent << "radius: ";
    Printer<float>::stream(s, indent + "  ", v.radius);
    s << indent << "top_left_x: ";
    Printer<float>::stream(s, indent + "  ", v.top_left_x);
    s << indent << "top_left_y: ";
    Printer<float>::stream(s, indent + "  ", v.top_left_y);
    s << indent << "bottom_right_x: ";
    Printer<float>::stream(s, indent + "  ", v.bottom_right_x);
    s << indent << "bottom_right_y: ";
    Printer<float>::stream(s, indent + "  ", v.bottom_right_y);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MAVROS_MSGS_MESSAGE_GIMBALMANAGERCAMERATRACKREQUEST_H
