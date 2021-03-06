/* Auto-generated by genmsg_cpp for file /home/sakai_nuc/rosbuild_ws/sandbox/amu_3002a_lite/srv/amu_operator.srv */
#ifndef AMU_3002A_LITE_SERVICE_AMU_OPERATOR_H
#define AMU_3002A_LITE_SERVICE_AMU_OPERATOR_H
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/time.h"

#include "ros/macros.h"

#include "ros/assert.h"

#include "ros/service_traits.h"




namespace amu_3002a_lite
{
template <class ContainerAllocator>
struct amu_operatorRequest_ {
  typedef amu_operatorRequest_<ContainerAllocator> Type;

  amu_operatorRequest_()
  : command(0)
  {
  }

  amu_operatorRequest_(const ContainerAllocator& _alloc)
  : command(0)
  {
  }

  typedef int8_t _command_type;
  int8_t command;


  typedef boost::shared_ptr< ::amu_3002a_lite::amu_operatorRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::amu_3002a_lite::amu_operatorRequest_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct amu_operatorRequest
typedef  ::amu_3002a_lite::amu_operatorRequest_<std::allocator<void> > amu_operatorRequest;

typedef boost::shared_ptr< ::amu_3002a_lite::amu_operatorRequest> amu_operatorRequestPtr;
typedef boost::shared_ptr< ::amu_3002a_lite::amu_operatorRequest const> amu_operatorRequestConstPtr;



template <class ContainerAllocator>
struct amu_operatorResponse_ {
  typedef amu_operatorResponse_<ContainerAllocator> Type;

  amu_operatorResponse_()
  {
  }

  amu_operatorResponse_(const ContainerAllocator& _alloc)
  {
  }


  typedef boost::shared_ptr< ::amu_3002a_lite::amu_operatorResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::amu_3002a_lite::amu_operatorResponse_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct amu_operatorResponse
typedef  ::amu_3002a_lite::amu_operatorResponse_<std::allocator<void> > amu_operatorResponse;

typedef boost::shared_ptr< ::amu_3002a_lite::amu_operatorResponse> amu_operatorResponsePtr;
typedef boost::shared_ptr< ::amu_3002a_lite::amu_operatorResponse const> amu_operatorResponseConstPtr;


struct amu_operator
{

typedef amu_operatorRequest Request;
typedef amu_operatorResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct amu_operator
} // namespace amu_3002a_lite

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::amu_3002a_lite::amu_operatorRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::amu_3002a_lite::amu_operatorRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::amu_3002a_lite::amu_operatorRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "481ac5a494c3140a2539020bd74c82c7";
  }

  static const char* value(const  ::amu_3002a_lite::amu_operatorRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x481ac5a494c3140aULL;
  static const uint64_t static_value2 = 0x2539020bd74c82c7ULL;
};

template<class ContainerAllocator>
struct DataType< ::amu_3002a_lite::amu_operatorRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "amu_3002a_lite/amu_operatorRequest";
  }

  static const char* value(const  ::amu_3002a_lite::amu_operatorRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::amu_3002a_lite::amu_operatorRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "int8 command\n\
\n\
";
  }

  static const char* value(const  ::amu_3002a_lite::amu_operatorRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::amu_3002a_lite::amu_operatorRequest_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::amu_3002a_lite::amu_operatorResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::amu_3002a_lite::amu_operatorResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::amu_3002a_lite::amu_operatorResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const  ::amu_3002a_lite::amu_operatorResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::amu_3002a_lite::amu_operatorResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "amu_3002a_lite/amu_operatorResponse";
  }

  static const char* value(const  ::amu_3002a_lite::amu_operatorResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::amu_3002a_lite::amu_operatorResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "\n\
\n\
\n\
";
  }

  static const char* value(const  ::amu_3002a_lite::amu_operatorResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::amu_3002a_lite::amu_operatorResponse_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::amu_3002a_lite::amu_operatorRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.command);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct amu_operatorRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::amu_3002a_lite::amu_operatorResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct amu_operatorResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<amu_3002a_lite::amu_operator> {
  static const char* value() 
  {
    return "481ac5a494c3140a2539020bd74c82c7";
  }

  static const char* value(const amu_3002a_lite::amu_operator&) { return value(); } 
};

template<>
struct DataType<amu_3002a_lite::amu_operator> {
  static const char* value() 
  {
    return "amu_3002a_lite/amu_operator";
  }

  static const char* value(const amu_3002a_lite::amu_operator&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<amu_3002a_lite::amu_operatorRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "481ac5a494c3140a2539020bd74c82c7";
  }

  static const char* value(const amu_3002a_lite::amu_operatorRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<amu_3002a_lite::amu_operatorRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "amu_3002a_lite/amu_operator";
  }

  static const char* value(const amu_3002a_lite::amu_operatorRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<amu_3002a_lite::amu_operatorResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "481ac5a494c3140a2539020bd74c82c7";
  }

  static const char* value(const amu_3002a_lite::amu_operatorResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<amu_3002a_lite::amu_operatorResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "amu_3002a_lite/amu_operator";
  }

  static const char* value(const amu_3002a_lite::amu_operatorResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // AMU_3002A_LITE_SERVICE_AMU_OPERATOR_H

