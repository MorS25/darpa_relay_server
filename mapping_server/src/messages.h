#pragma once

// ======================================================================
// Regular ROS types
// ======================================================================

#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>

// ======================================================================
// Synthetic types to help sequence map processing
// ======================================================================

struct MapUpdate
{
  std::string type;
  std::string name;
};
struct MapUpdateGrid
{
  nav_msgs::OccupancyGrid msg;
};
struct MapUpdateCloud
{
  sensor_msgs::PointCloud2 msg;
};
struct PointCloud2Origin
{
  geometry_msgs::Pose origin;
};
struct MapUpdateCloudOrigin
{
  PointCloud2Origin msg;
};

// ======================================================================
// Helper types for telemetry processing
// ======================================================================
struct TelemetryName
{
  std::string name;
};
struct TelemetryNames
{
  std::vector<TelemetryName> poses;
};

