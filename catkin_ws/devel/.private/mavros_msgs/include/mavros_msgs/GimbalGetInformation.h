// Generated by gencpp from file mavros_msgs/GimbalGetInformation.msg
// DO NOT EDIT!


#ifndef MAVROS_MSGS_MESSAGE_GIMBALGETINFORMATION_H
#define MAVROS_MSGS_MESSAGE_GIMBALGETINFORMATION_H

#include <ros/service_traits.h>


#include <mavros_msgs/GimbalGetInformationRequest.h>
#include <mavros_msgs/GimbalGetInformationResponse.h>


namespace mavros_msgs
{

struct GimbalGetInformation
{

typedef GimbalGetInformationRequest Request;
typedef GimbalGetInformationResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct GimbalGetInformation
} // namespace mavros_msgs


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::mavros_msgs::GimbalGetInformation > {
  static const char* value()
  {
    return "1cd894375e4e3d2861d2222772894fdb";
  }

  static const char* value(const ::mavros_msgs::GimbalGetInformation&) { return value(); }
};

template<>
struct DataType< ::mavros_msgs::GimbalGetInformation > {
  static const char* value()
  {
    return "mavros_msgs/GimbalGetInformation";
  }

  static const char* value(const ::mavros_msgs::GimbalGetInformation&) { return value(); }
};


// service_traits::MD5Sum< ::mavros_msgs::GimbalGetInformationRequest> should match
// service_traits::MD5Sum< ::mavros_msgs::GimbalGetInformation >
template<>
struct MD5Sum< ::mavros_msgs::GimbalGetInformationRequest>
{
  static const char* value()
  {
    return MD5Sum< ::mavros_msgs::GimbalGetInformation >::value();
  }
  static const char* value(const ::mavros_msgs::GimbalGetInformationRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::mavros_msgs::GimbalGetInformationRequest> should match
// service_traits::DataType< ::mavros_msgs::GimbalGetInformation >
template<>
struct DataType< ::mavros_msgs::GimbalGetInformationRequest>
{
  static const char* value()
  {
    return DataType< ::mavros_msgs::GimbalGetInformation >::value();
  }
  static const char* value(const ::mavros_msgs::GimbalGetInformationRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::mavros_msgs::GimbalGetInformationResponse> should match
// service_traits::MD5Sum< ::mavros_msgs::GimbalGetInformation >
template<>
struct MD5Sum< ::mavros_msgs::GimbalGetInformationResponse>
{
  static const char* value()
  {
    return MD5Sum< ::mavros_msgs::GimbalGetInformation >::value();
  }
  static const char* value(const ::mavros_msgs::GimbalGetInformationResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::mavros_msgs::GimbalGetInformationResponse> should match
// service_traits::DataType< ::mavros_msgs::GimbalGetInformation >
template<>
struct DataType< ::mavros_msgs::GimbalGetInformationResponse>
{
  static const char* value()
  {
    return DataType< ::mavros_msgs::GimbalGetInformation >::value();
  }
  static const char* value(const ::mavros_msgs::GimbalGetInformationResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // MAVROS_MSGS_MESSAGE_GIMBALGETINFORMATION_H
