// Generated by gencpp from file mavros_msgs/GimbalManagerConfigure.msg
// DO NOT EDIT!


#ifndef MAVROS_MSGS_MESSAGE_GIMBALMANAGERCONFIGURE_H
#define MAVROS_MSGS_MESSAGE_GIMBALMANAGERCONFIGURE_H

#include <ros/service_traits.h>


#include <mavros_msgs/GimbalManagerConfigureRequest.h>
#include <mavros_msgs/GimbalManagerConfigureResponse.h>


namespace mavros_msgs
{

struct GimbalManagerConfigure
{

typedef GimbalManagerConfigureRequest Request;
typedef GimbalManagerConfigureResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct GimbalManagerConfigure
} // namespace mavros_msgs


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::mavros_msgs::GimbalManagerConfigure > {
  static const char* value()
  {
    return "33077e29a7a8218a30def4cf603efdb4";
  }

  static const char* value(const ::mavros_msgs::GimbalManagerConfigure&) { return value(); }
};

template<>
struct DataType< ::mavros_msgs::GimbalManagerConfigure > {
  static const char* value()
  {
    return "mavros_msgs/GimbalManagerConfigure";
  }

  static const char* value(const ::mavros_msgs::GimbalManagerConfigure&) { return value(); }
};


// service_traits::MD5Sum< ::mavros_msgs::GimbalManagerConfigureRequest> should match
// service_traits::MD5Sum< ::mavros_msgs::GimbalManagerConfigure >
template<>
struct MD5Sum< ::mavros_msgs::GimbalManagerConfigureRequest>
{
  static const char* value()
  {
    return MD5Sum< ::mavros_msgs::GimbalManagerConfigure >::value();
  }
  static const char* value(const ::mavros_msgs::GimbalManagerConfigureRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::mavros_msgs::GimbalManagerConfigureRequest> should match
// service_traits::DataType< ::mavros_msgs::GimbalManagerConfigure >
template<>
struct DataType< ::mavros_msgs::GimbalManagerConfigureRequest>
{
  static const char* value()
  {
    return DataType< ::mavros_msgs::GimbalManagerConfigure >::value();
  }
  static const char* value(const ::mavros_msgs::GimbalManagerConfigureRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::mavros_msgs::GimbalManagerConfigureResponse> should match
// service_traits::MD5Sum< ::mavros_msgs::GimbalManagerConfigure >
template<>
struct MD5Sum< ::mavros_msgs::GimbalManagerConfigureResponse>
{
  static const char* value()
  {
    return MD5Sum< ::mavros_msgs::GimbalManagerConfigure >::value();
  }
  static const char* value(const ::mavros_msgs::GimbalManagerConfigureResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::mavros_msgs::GimbalManagerConfigureResponse> should match
// service_traits::DataType< ::mavros_msgs::GimbalManagerConfigure >
template<>
struct DataType< ::mavros_msgs::GimbalManagerConfigureResponse>
{
  static const char* value()
  {
    return DataType< ::mavros_msgs::GimbalManagerConfigure >::value();
  }
  static const char* value(const ::mavros_msgs::GimbalManagerConfigureResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // MAVROS_MSGS_MESSAGE_GIMBALMANAGERCONFIGURE_H
