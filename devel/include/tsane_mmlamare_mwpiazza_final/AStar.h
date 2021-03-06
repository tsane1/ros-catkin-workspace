// Generated by gencpp from file tsane_mmlamare_mwpiazza_final/AStar.msg
// DO NOT EDIT!


#ifndef TSANE_MMLAMARE_MWPIAZZA_FINAL_MESSAGE_ASTAR_H
#define TSANE_MMLAMARE_MWPIAZZA_FINAL_MESSAGE_ASTAR_H

#include <ros/service_traits.h>


#include <tsane_mmlamare_mwpiazza_final/AStarRequest.h>
#include <tsane_mmlamare_mwpiazza_final/AStarResponse.h>


namespace tsane_mmlamare_mwpiazza_final
{

struct AStar
{

typedef AStarRequest Request;
typedef AStarResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct AStar
} // namespace tsane_mmlamare_mwpiazza_final


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::tsane_mmlamare_mwpiazza_final::AStar > {
  static const char* value()
  {
    return "83917d80c2c524ed3baa8ca42ce18e66";
  }

  static const char* value(const ::tsane_mmlamare_mwpiazza_final::AStar&) { return value(); }
};

template<>
struct DataType< ::tsane_mmlamare_mwpiazza_final::AStar > {
  static const char* value()
  {
    return "tsane_mmlamare_mwpiazza_final/AStar";
  }

  static const char* value(const ::tsane_mmlamare_mwpiazza_final::AStar&) { return value(); }
};


// service_traits::MD5Sum< ::tsane_mmlamare_mwpiazza_final::AStarRequest> should match 
// service_traits::MD5Sum< ::tsane_mmlamare_mwpiazza_final::AStar > 
template<>
struct MD5Sum< ::tsane_mmlamare_mwpiazza_final::AStarRequest>
{
  static const char* value()
  {
    return MD5Sum< ::tsane_mmlamare_mwpiazza_final::AStar >::value();
  }
  static const char* value(const ::tsane_mmlamare_mwpiazza_final::AStarRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::tsane_mmlamare_mwpiazza_final::AStarRequest> should match 
// service_traits::DataType< ::tsane_mmlamare_mwpiazza_final::AStar > 
template<>
struct DataType< ::tsane_mmlamare_mwpiazza_final::AStarRequest>
{
  static const char* value()
  {
    return DataType< ::tsane_mmlamare_mwpiazza_final::AStar >::value();
  }
  static const char* value(const ::tsane_mmlamare_mwpiazza_final::AStarRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::tsane_mmlamare_mwpiazza_final::AStarResponse> should match 
// service_traits::MD5Sum< ::tsane_mmlamare_mwpiazza_final::AStar > 
template<>
struct MD5Sum< ::tsane_mmlamare_mwpiazza_final::AStarResponse>
{
  static const char* value()
  {
    return MD5Sum< ::tsane_mmlamare_mwpiazza_final::AStar >::value();
  }
  static const char* value(const ::tsane_mmlamare_mwpiazza_final::AStarResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::tsane_mmlamare_mwpiazza_final::AStarResponse> should match 
// service_traits::DataType< ::tsane_mmlamare_mwpiazza_final::AStar > 
template<>
struct DataType< ::tsane_mmlamare_mwpiazza_final::AStarResponse>
{
  static const char* value()
  {
    return DataType< ::tsane_mmlamare_mwpiazza_final::AStar >::value();
  }
  static const char* value(const ::tsane_mmlamare_mwpiazza_final::AStarResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // TSANE_MMLAMARE_MWPIAZZA_FINAL_MESSAGE_ASTAR_H
