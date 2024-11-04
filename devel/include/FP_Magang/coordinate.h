// Generated by gencpp from file FP_Magang/coordinate.msg
// DO NOT EDIT!


#ifndef FP_MAGANG_MESSAGE_COORDINATE_H
#define FP_MAGANG_MESSAGE_COORDINATE_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace FP_Magang
{
template <class ContainerAllocator>
struct coordinate_
{
  typedef coordinate_<ContainerAllocator> Type;

  coordinate_()
    : x(0.0)
    , y(0.0)
    , z(0.0)  {
    }
  coordinate_(const ContainerAllocator& _alloc)
    : x(0.0)
    , y(0.0)
    , z(0.0)  {
  (void)_alloc;
    }



   typedef float _x_type;
  _x_type x;

   typedef float _y_type;
  _y_type y;

   typedef float _z_type;
  _z_type z;





  typedef boost::shared_ptr< ::FP_Magang::coordinate_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::FP_Magang::coordinate_<ContainerAllocator> const> ConstPtr;

}; // struct coordinate_

typedef ::FP_Magang::coordinate_<std::allocator<void> > coordinate;

typedef boost::shared_ptr< ::FP_Magang::coordinate > coordinatePtr;
typedef boost::shared_ptr< ::FP_Magang::coordinate const> coordinateConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::FP_Magang::coordinate_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::FP_Magang::coordinate_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::FP_Magang::coordinate_<ContainerAllocator1> & lhs, const ::FP_Magang::coordinate_<ContainerAllocator2> & rhs)
{
  return lhs.x == rhs.x &&
    lhs.y == rhs.y &&
    lhs.z == rhs.z;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::FP_Magang::coordinate_<ContainerAllocator1> & lhs, const ::FP_Magang::coordinate_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace FP_Magang

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::FP_Magang::coordinate_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::FP_Magang::coordinate_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::FP_Magang::coordinate_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::FP_Magang::coordinate_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::FP_Magang::coordinate_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::FP_Magang::coordinate_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::FP_Magang::coordinate_<ContainerAllocator> >
{
  static const char* value()
  {
    return "cc153912f1453b708d221682bc23d9ac";
  }

  static const char* value(const ::FP_Magang::coordinate_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xcc153912f1453b70ULL;
  static const uint64_t static_value2 = 0x8d221682bc23d9acULL;
};

template<class ContainerAllocator>
struct DataType< ::FP_Magang::coordinate_<ContainerAllocator> >
{
  static const char* value()
  {
    return "FP_Magang/coordinate";
  }

  static const char* value(const ::FP_Magang::coordinate_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::FP_Magang::coordinate_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float32 x\n"
"float32 y\n"
"float32 z\n"
;
  }

  static const char* value(const ::FP_Magang::coordinate_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::FP_Magang::coordinate_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.x);
      stream.next(m.y);
      stream.next(m.z);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct coordinate_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::FP_Magang::coordinate_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::FP_Magang::coordinate_<ContainerAllocator>& v)
  {
    s << indent << "x: ";
    Printer<float>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<float>::stream(s, indent + "  ", v.y);
    s << indent << "z: ";
    Printer<float>::stream(s, indent + "  ", v.z);
  }
};

} // namespace message_operations
} // namespace ros

#endif // FP_MAGANG_MESSAGE_COORDINATE_H
