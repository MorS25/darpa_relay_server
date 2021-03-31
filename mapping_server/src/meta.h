#pragma once

#include <metastuff/Forward.h>
#include <metastuff/Json.h>
#include "messages.h"

// ======================================================================
// std_msgs types
// ======================================================================

namespace std_msgs
{
  #include <metastuff/JsonNamespace.h>
}

namespace meta
{
  using namespace std_msgs;
  using namespace geometry_msgs;
  using namespace nav_msgs;
  using namespace sensor_msgs;

  template <>
  inline auto registerMembers<Header>()
  {
    return members
      ( member("seq",      &Header::seq,      true)
      , member("stamp",    &Header::stamp,    true)
      , member("frame_id", &Header::frame_id, true)
      );
  }
}

// ======================================================================
// geometry_msgs types
// ======================================================================

namespace geometry_msgs
{
  #include <metastuff/JsonNamespace.h>
}
namespace visualization_msgs
{
  #include <metastuff/JsonNamespace.h>
}


namespace meta
{
  using namespace geometry_msgs;

  template <>
  inline auto registerMembers<Point>()
  {
    return members
      ( member("x", &Point::x)
      , member("y", &Point::y)
      , member("z", &Point::z)
      );
  }

  template <>
  inline auto registerMembers<Vector3>()
  {
    return members
      ( member("x", &Vector3::x)
      , member("y", &Vector3::y)
      , member("z", &Vector3::z)
      );
  }

  template <>
  inline auto registerMembers<Quaternion>()
  {
    return members
      ( member("x", &Quaternion::x)
      , member("y", &Quaternion::y)
      , member("z", &Quaternion::z)
      , member("w", &Quaternion::w)
      );
  }

  template <>
  inline auto registerMembers<Pose>()
  {
    return members
      ( member("position",    &Pose::position   )  
      , member("orientation", &Pose::orientation)
      );
  }

  template <>
  inline auto registerMembers<PoseArray>()
  {
    return members
      ( member("header",      &PoseArray::header )
      , member("poses",       &PoseArray::poses        )
      );
  }
}
namespace meta
{
  using namespace std_msgs;
  template <>
  inline auto registerMembers<ColorRGBA>()
  {
    return members
      ( member("r", &ColorRGBA::r)
      , member("g", &ColorRGBA::g)
      , member("b", &ColorRGBA::b)
      , member("a", &ColorRGBA::a)
      );
  }

}
namespace meta
{
  using namespace visualization_msgs;
  using namespace std_msgs;
  using namespace geometry_msgs;
  template <>
  inline auto registerMembers<Marker>()
  {
    return members
      ( member("ns", &Marker::ns)
        , member("id",  &Marker::id)
        , member("type", &Marker::type)
        , member("action", &Marker::action)
        , member("header",    &Marker::header, true)
        , member("pose"  ,    &Marker::pose)
        , member("scale",     &Marker::scale, true)
        , member("color",     &Marker::color, true)
        , member("points",    &Marker::points, true)
        , member("colors",    &Marker::colors, true)
        , member("text",      &Marker::text, true)
        , member("mesh_resource", &Marker::mesh_resource, true)
        , member("mesh_use_embedded_materials", &Marker::mesh_use_embedded_materials, true)
        );
    }
  template <>
  inline auto registerMembers<MarkerArray>()
  {
    return members
      ( member("markers",     &MarkerArray::markers)
      );
  }
}

 
// ======================================================================
// nav_msgs types
// ======================================================================

namespace nav_msgs
{
  #include <metastuff/JsonNamespace.h>
}

namespace meta
{
  using namespace nav_msgs;

  template <>
  inline auto registerMembers<MapMetaData>()
  {
    return members
      ( member("map_load_time", &MapMetaData::map_load_time, true)
      , member("resolution",    &MapMetaData::resolution         )
      , member("width",         &MapMetaData::width              )
      , member("height",        &MapMetaData::height             )
      , member("origin",        &MapMetaData::origin             )
      );
  }
  
  template <>
  inline auto registerMembers<OccupancyGrid>()
  {
    return members
      ( member("header",  &OccupancyGrid::header, true)
      , member("info",    &OccupancyGrid::info)
      );
  }
}

// ======================================================================
// sensor_msgs types
// ======================================================================

namespace sensor_msgs
{
  #include <metastuff/JsonNamespace.h>
}

namespace meta
{
  using namespace sensor_msgs;

  template <>
  inline auto registerMembers<PointField>()
  {
    return members
      ( member("name",     &PointField::name)
      , member("offset",   &PointField::offset)
      , member("datatype", &PointField::datatype)
      , member("count",    &PointField::count)
      );
  }

  template <>
  inline auto registerMembers<PointCloud2>()
  {
    return members
      ( member("header",       &PointCloud2::header,       true)
      , member("fields",       &PointCloud2::fields)
      , member("is_bigendian", &PointCloud2::is_bigendian, true)
      , member("point_step",   &PointCloud2::point_step)
      );
  }
}

// ======================================================================
// map processing synthetic types
// ======================================================================

namespace meta
{
  template <>
  inline auto registerMembers<MapUpdate>()
  {
    return members
      ( member("type", &MapUpdate::type)
      , member("name", &MapUpdate::name)
      );
  }

  template <>
  inline auto registerMembers<MapUpdateGrid>()
  {
    return members
      ( member("msg", &MapUpdateGrid::msg)
      );
  }

  template <>
  inline auto registerMembers<MapUpdateCloud>()
  {
    return members
      ( member("msg", &MapUpdateCloud::msg)
      );
  }

  template <>
  inline auto registerMembers<PointCloud2Origin>()
  {
    return members
      ( member("origin", &PointCloud2Origin::origin, true)
      );
  }

  template <>
  inline auto registerMembers<MapUpdateCloudOrigin>()
  {
    return members
      ( member("msg", &MapUpdateCloudOrigin::msg)
      );
  }
}

// ======================================================================
// map processing synthetic types
// ======================================================================

namespace meta
{
  template <>
  inline auto registerMembers<TelemetryName>()
  {
    return members
      ( member("name", &TelemetryName::name, true)
      );
  }

  template <>
  inline auto registerMembers<TelemetryNames>()
  {
    return members
      ( member("poses", &TelemetryNames::poses)
      );
  }
}
