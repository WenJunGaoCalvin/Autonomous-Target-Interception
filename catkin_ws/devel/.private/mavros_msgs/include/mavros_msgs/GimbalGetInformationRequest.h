// Generated by gencpp from file mavros_msgs/GimbalGetInformationRequest.msg
// DO NOT EDIT!


#ifndef MAVROS_MSGS_MESSAGE_GIMBALGETINFORMATIONREQUEST_H
#define MAVROS_MSGS_MESSAGE_GIMBALGETINFORMATIONREQUEST_H


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
struct GimbalGetInformationRequest_
{
  typedef GimbalGetInformationRequest_<ContainerAllocator> Type;

  GimbalGetInformationRequest_()
    {
    }
  GimbalGetInformationRequest_(const ContainerAllocator& _alloc)
    {
  (void)_alloc;
    }







  typedef boost::shared_ptr< ::mavros_msgs::GimbalGetInformationRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::mavros_msgs::GimbalGetInformationRequest_<ContainerAllocator> const> ConstPtr;

}; // struct GimbalGetInformationRequest_

typedef ::mavros_msgs::GimbalGetInformationRequest_<std::allocator<void> > GimbalGetInformationRequest;

typedef boost::shared_ptr< ::mavros_msgs::GimbalGetInformationRequest > GimbalGetInformationRequestPtr;
typedef boost::shared_ptr< ::mavros_msgs::GimbalGetInformationRequest const> GimbalGetInformationRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::mavros_msgs::GimbalGetInformationRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::mavros_msgs::GimbalGetInformationRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


} // namespace mavros_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::mavros_msgs::GimbalGetInformationRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mavros_msgs::GimbalGetInformationRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::mavros_msgs::GimbalGetInformationRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::mavros_msgs::GimbalGetInformationRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mavros_msgs::GimbalGetInformationRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mavros_msgs::GimbalGetInformationRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::mavros_msgs::GimbalGetInformationRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const ::mavros_msgs::GimbalGetInformationRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::mavros_msgs::GimbalGetInformationRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "mavros_msgs/GimbalGetInformationRequest";
  }

  static const char* value(const ::mavros_msgs::GimbalGetInformationRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::mavros_msgs::GimbalGetInformationRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# MAVLink command: MAV_CMD_REQUEST_MESSAGE\n"
"# https://mavlink.io/en/messages/common.html#MAV_CMD_REQUEST_MESSAGE\n"
"# Specifically used to request Information messages from Gimbal Device and Gimbal Manager\n"
"# https://mavlink.io/en/messages/common.html#GIMBAL_MANAGER_INFORMATION\n"
"# https://mavlink.io/en/messages/common.html#GIMBAL_DEVICE_INFORMATION\n"
"\n"
;
  }

  static const char* value(const ::mavros_msgs::GimbalGetInformationRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::mavros_msgs::GimbalGetInformationRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream&, T)
    {}

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GimbalGetInformationRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::mavros_msgs::GimbalGetInformationRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream&, const std::string&, const ::mavros_msgs::GimbalGetInformationRequest_<ContainerAllocator>&)
  {}
};

} // namespace message_operations
} // namespace ros

#endif // MAVROS_MSGS_MESSAGE_GIMBALGETINFORMATIONREQUEST_H