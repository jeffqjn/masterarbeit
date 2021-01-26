// Generated by gencpp from file velocity2state2/truth_rotation.msg
// DO NOT EDIT!


#ifndef VELOCITY2STATE2_MESSAGE_TRUTH_ROTATION_H
#define VELOCITY2STATE2_MESSAGE_TRUTH_ROTATION_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <tf/tfMessage.h>

namespace velocity2state2
{
template <class ContainerAllocator>
struct truth_rotation_
{
  typedef truth_rotation_<ContainerAllocator> Type;

  truth_rotation_()
    : send_transform()  {
    }
  truth_rotation_(const ContainerAllocator& _alloc)
    : send_transform(_alloc)  {
  (void)_alloc;
    }



   typedef  ::tf::tfMessage_<ContainerAllocator>  _send_transform_type;
  _send_transform_type send_transform;





  typedef boost::shared_ptr< ::velocity2state2::truth_rotation_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::velocity2state2::truth_rotation_<ContainerAllocator> const> ConstPtr;

}; // struct truth_rotation_

typedef ::velocity2state2::truth_rotation_<std::allocator<void> > truth_rotation;

typedef boost::shared_ptr< ::velocity2state2::truth_rotation > truth_rotationPtr;
typedef boost::shared_ptr< ::velocity2state2::truth_rotation const> truth_rotationConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::velocity2state2::truth_rotation_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::velocity2state2::truth_rotation_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::velocity2state2::truth_rotation_<ContainerAllocator1> & lhs, const ::velocity2state2::truth_rotation_<ContainerAllocator2> & rhs)
{
  return lhs.send_transform == rhs.send_transform;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::velocity2state2::truth_rotation_<ContainerAllocator1> & lhs, const ::velocity2state2::truth_rotation_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace velocity2state2

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::velocity2state2::truth_rotation_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::velocity2state2::truth_rotation_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::velocity2state2::truth_rotation_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::velocity2state2::truth_rotation_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::velocity2state2::truth_rotation_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::velocity2state2::truth_rotation_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::velocity2state2::truth_rotation_<ContainerAllocator> >
{
  static const char* value()
  {
    return "57c643213939b2acaf9cdeb59405057e";
  }

  static const char* value(const ::velocity2state2::truth_rotation_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x57c643213939b2acULL;
  static const uint64_t static_value2 = 0xaf9cdeb59405057eULL;
};

template<class ContainerAllocator>
struct DataType< ::velocity2state2::truth_rotation_<ContainerAllocator> >
{
  static const char* value()
  {
    return "velocity2state2/truth_rotation";
  }

  static const char* value(const ::velocity2state2::truth_rotation_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::velocity2state2::truth_rotation_<ContainerAllocator> >
{
  static const char* value()
  {
    return "tf/tfMessage send_transform\n"
"\n"
"================================================================================\n"
"MSG: tf/tfMessage\n"
"geometry_msgs/TransformStamped[] transforms\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/TransformStamped\n"
"# This expresses a transform from coordinate frame header.frame_id\n"
"# to the coordinate frame child_frame_id\n"
"#\n"
"# This message is mostly used by the \n"
"# <a href=\"http://wiki.ros.org/tf\">tf</a> package. \n"
"# See its documentation for more information.\n"
"\n"
"Header header\n"
"string child_frame_id # the frame id of the child frame\n"
"Transform transform\n"
"\n"
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
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Transform\n"
"# This represents the transform between two coordinate frames in free space.\n"
"\n"
"Vector3 translation\n"
"Quaternion rotation\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Vector3\n"
"# This represents a vector in free space. \n"
"# It is only meant to represent a direction. Therefore, it does not\n"
"# make sense to apply a translation to it (e.g., when applying a \n"
"# generic rigid transformation to a Vector3, tf2 will only apply the\n"
"# rotation). If you want your data to be translatable too, use the\n"
"# geometry_msgs/Point message instead.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
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

  static const char* value(const ::velocity2state2::truth_rotation_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::velocity2state2::truth_rotation_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.send_transform);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct truth_rotation_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::velocity2state2::truth_rotation_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::velocity2state2::truth_rotation_<ContainerAllocator>& v)
  {
    s << indent << "send_transform: ";
    s << std::endl;
    Printer< ::tf::tfMessage_<ContainerAllocator> >::stream(s, indent + "  ", v.send_transform);
  }
};

} // namespace message_operations
} // namespace ros

#endif // VELOCITY2STATE2_MESSAGE_TRUTH_ROTATION_H
