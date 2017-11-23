// Generated by gencpp from file pedestrian_tracking/PedestrianPose.msg
// DO NOT EDIT!


#ifndef PEDESTRIAN_TRACKING_MESSAGE_PEDESTRIANPOSE_H
#define PEDESTRIAN_TRACKING_MESSAGE_PEDESTRIANPOSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace pedestrian_tracking
{
template <class ContainerAllocator>
struct PedestrianPose_
{
  typedef PedestrianPose_<ContainerAllocator> Type;

  PedestrianPose_()
    : pedID(0)
    , frameID(0)
    , x(0.0)
    , y(0.0)  {
    }
  PedestrianPose_(const ContainerAllocator& _alloc)
    : pedID(0)
    , frameID(0)
    , x(0.0)
    , y(0.0)  {
  (void)_alloc;
    }



   typedef uint32_t _pedID_type;
  _pedID_type pedID;

   typedef uint32_t _frameID_type;
  _frameID_type frameID;

   typedef double _x_type;
  _x_type x;

   typedef double _y_type;
  _y_type y;




  typedef boost::shared_ptr< ::pedestrian_tracking::PedestrianPose_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::pedestrian_tracking::PedestrianPose_<ContainerAllocator> const> ConstPtr;

}; // struct PedestrianPose_

typedef ::pedestrian_tracking::PedestrianPose_<std::allocator<void> > PedestrianPose;

typedef boost::shared_ptr< ::pedestrian_tracking::PedestrianPose > PedestrianPosePtr;
typedef boost::shared_ptr< ::pedestrian_tracking::PedestrianPose const> PedestrianPoseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::pedestrian_tracking::PedestrianPose_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::pedestrian_tracking::PedestrianPose_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace pedestrian_tracking

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'pedestrian_tracking': ['/home/teame16/CMUSelfDrivingCar/src/perception/pedestrian_tracking/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::pedestrian_tracking::PedestrianPose_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::pedestrian_tracking::PedestrianPose_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pedestrian_tracking::PedestrianPose_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pedestrian_tracking::PedestrianPose_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pedestrian_tracking::PedestrianPose_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pedestrian_tracking::PedestrianPose_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::pedestrian_tracking::PedestrianPose_<ContainerAllocator> >
{
  static const char* value()
  {
    return "8c6b22c503c8ea4c695da904ced715cd";
  }

  static const char* value(const ::pedestrian_tracking::PedestrianPose_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x8c6b22c503c8ea4cULL;
  static const uint64_t static_value2 = 0x695da904ced715cdULL;
};

template<class ContainerAllocator>
struct DataType< ::pedestrian_tracking::PedestrianPose_<ContainerAllocator> >
{
  static const char* value()
  {
    return "pedestrian_tracking/PedestrianPose";
  }

  static const char* value(const ::pedestrian_tracking::PedestrianPose_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::pedestrian_tracking::PedestrianPose_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint32 pedID\n\
uint32 frameID\n\
float64 x\n\
float64 y\n\
\n\
#undecided data type\n\
";
  }

  static const char* value(const ::pedestrian_tracking::PedestrianPose_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::pedestrian_tracking::PedestrianPose_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.pedID);
      stream.next(m.frameID);
      stream.next(m.x);
      stream.next(m.y);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct PedestrianPose_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::pedestrian_tracking::PedestrianPose_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::pedestrian_tracking::PedestrianPose_<ContainerAllocator>& v)
  {
    s << indent << "pedID: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.pedID);
    s << indent << "frameID: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.frameID);
    s << indent << "x: ";
    Printer<double>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<double>::stream(s, indent + "  ", v.y);
  }
};

} // namespace message_operations
} // namespace ros

#endif // PEDESTRIAN_TRACKING_MESSAGE_PEDESTRIANPOSE_H
