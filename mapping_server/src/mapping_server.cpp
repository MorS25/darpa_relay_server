#include <ros/ros.h>

// ROS message types and support types
#include "messages.h"
#include <tf2_msgs/TFMessage.h>

// HTTP server implementation
#include <restinio/all.hpp>
#include "level_logger.hpp"
#include "http.h"
#include <fmt/format.h>
#include <fmt/ostream.h>

// Compression tools
#include <restinio/transforms/zlib.hpp>

// Numeric tools for manipulating point clouds
#include <Eigen/Geometry>
#include <geometry_msgs/Transform.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/point_cloud_conversion.h>

// Our JSON/CBOR conversion routines
#include <metastuff/Json.h>
#include <metastuff/Cbor.h>
#include "meta.h"

using nlohmann::json;
using router_t = restinio::router::express_router_t<>;

// --------------------------------------------------
// Map message wrapper
// --------------------------------------------------

std::string flatten_nested_exception(const std::exception& e)
{
  std::stringstream ss;
  int level = 0;
  std::function<void(const std::exception&)> inner =
    [&](const std::exception& e)
    {
      ss << std::string(2*level++, ' ') << e.what() << '\n';
      try {
        std::rethrow_if_nested(e);
      } catch (const std::exception& e) {
        inner(e);
      } catch(...) {}
    };
  inner(e);
  return ss.str();
}

// ======================================================================
// Binary data handling
// ======================================================================

template <typename T, typename Container,
         typename = std::enable_if_t<sizeof(T) == sizeof(typename Container::value_type)>>
std::vector<T> decompress( const Container& input, const restinio::transforms::zlib::params_t& params )
{
  using restinio::string_view_t;
  auto output = restinio::transforms::zlib::transform(
      string_view_t( reinterpret_cast<const string_view_t::value_type *>( input.data() ), input.size() ),
      params
      );
  return std::vector<T>(output.begin(), output.end());
}

template <typename T, typename Container>
std::vector<T> decompress_gzip(const Container& input)
{
  return decompress<T>(input, restinio::transforms::zlib::make_gzip_decompress_params() );
}

template <typename T, typename Container>
std::vector<T> decompress_deflate(const Container& input)
{
  return decompress<T>(input, restinio::transforms::zlib::make_deflate_decompress_params() );
}

// ======================================================================
// JSON unpacking routines
// ======================================================================

static std::string base64_encode(const std::string &in) {

    std::string out;

    int val=0, valb=-6;
    for (unsigned char c : in) {
        val = (val<<8) + c;
        valb += 8;
        while (valb>=0) {
            out.push_back("ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/"[(val>>valb)&0x3F]);
            valb-=6;
        }
    }
    if (valb>-6) out.push_back("ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/"[((val<<8)>>(valb+8))&0x3F]);
    while (out.size()%4) out.push_back('=');
    return out;
}

static std::string base64_decode(const std::string &in) {

    std::string out;

    std::vector<int> T(256,-1);
    for (int i=0; i<64; i++) T["ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/"[i]] = i; 

    int val=0, valb=-8;
    for (unsigned char c : in) {
        if (T[c] == -1) break;
        val = (val<<6) + T[c];
        valb += 6;
        if (valb>=0) {
            out.push_back(char((val>>valb)&0xFF));
            valb-=8;
        }
    }
    return out;
}

template <typename BasicJsonType, typename T>
void from_json_binary(const BasicJsonType& j, std::vector<T>& data)
{
  // Decode the base64-encoded data
  const auto& binary64 = j.at("data").template get<std::string>();
  auto binary = base64_decode(binary64);

  // Decompress as necessary
  std::string compression = j.value("compression", "none");
  std::transform(compression.begin(), compression.end(), compression.begin(), ::tolower);
  if (compression == "none")
    data = std::vector<T>(binary.begin(), binary.end());
  else if (compression == "gzip")
    data = decompress_gzip<T>(binary);
  else
    throw std::runtime_error("Unknown data compression type \"" + compression + "\".");
}

namespace ros
{
  template <typename BasicJsonType>
  void from_json(const BasicJsonType& j, Time& t)
  {
    t.fromSec(j);
  }
}

namespace geometry_msgs
{
  template <typename BasicJsonType>
  void from_json(const BasicJsonType& j, PoseArray& t)
  {
    from_json_inner(j, t);
  }
}
namespace visualization_msgs
{
  template <typename BasicJsonType>
  void from_json(const BasicJsonType& j, MarkerArray& t)
  {
    from_json_inner(j, t);
  }
}
namespace nav_msgs
{
  template <typename BasicJsonType>
  void from_json(const BasicJsonType& j, OccupancyGrid& t)
  {
    from_json_inner(j, t);
    from_json_binary(j, t.data);
  }
}

namespace sensor_msgs
{
  template <typename BasicJsonType>
  void from_json(const BasicJsonType& j, PointCloud2& t)
  {
    from_json_inner(j, t);
    from_json_binary(j, t.data);
  }
}

template <typename T>
void unpack(const json& j, T& t)
{
  from_json(j,t);
}

// ======================================================================
// CBOR unpacking routines
// ======================================================================

template <typename BasicCborType, typename T>
void from_cbor_binary(const BasicCborType& c, std::vector<T>& data)
{
  auto& map = c.to_map();
  const cbor::binary *binary = nullptr;
  {
    auto it = map.find("data");
    if (it != map.end())
      try {
        binary = &it->second.to_binary();
      } catch (const std::exception& e) {
        std::throw_with_nested(
            std::runtime_error(
              std::string("Error in field \"data\":")
              ));
      }
    else
      throw std::runtime_error("Required field \"data\" not found.");
  }
  std::string compression = "none";
  {
    auto it = map.find("compression");
    if (it != map.end())
      try {
        compression = it->second.to_string();
      } catch (const std::exception& e) {
        std::throw_with_nested(
            std::runtime_error(
              std::string("Error in field \"compression\":")
              ));
      }
  }
  std::transform(compression.begin(), compression.end(), compression.begin(), ::tolower);
  if (compression == "none")
    data = std::vector<T>(binary->begin(), binary->end());
  else if (compression == "gzip")
    data = decompress_gzip<T>(*binary);
  else
    throw std::runtime_error("Unknown compression type \"" + compression + "\".");
}

template <typename BasicCborType>
void from_cbor(const BasicCborType& c, ros::Time& t)
{
  t.fromSec(c);
}

template <typename BasicCborType>
void from_cbor(const BasicCborType& c, nav_msgs::OccupancyGrid& t)
{
  from_cbor_inner(c, t);
  from_cbor_binary(c, t.data);
}

template <typename BasicCborType>
void from_cbor(const BasicCborType& c, sensor_msgs::PointCloud2& t)
{
  from_cbor_inner(c, t);
  from_cbor_binary(c, t.data);
}

template <typename T>
void unpack(const cbor& c, T& t)
{
  from_cbor(c,t);
}

// ======================================================================
// Map unpacking
// ======================================================================

// --------------------------------------------------
// Grid unpacking
// --------------------------------------------------

template <typename MsgType>
void unpack_grid(const MsgType& msg, MapUpdateGrid& grid)
{
  // ------------------------------
  // Default values
  // ------------------------------
  grid.msg.header.stamp = ros::Time::now();
  grid.msg.header.frame_id = "darpa";

  // ------------------------------
  // Unpack message
  // ------------------------------
  unpack(msg, grid);

  // ------------------------------
  // Validation
  // ------------------------------
  if (grid.msg.header.frame_id != "darpa") {
    std::cout << "Got " << grid.msg.header.frame_id << " instead of darpa frame" << std::endl;
  }

  //if (grid.msg.header.frame_id != "darpa")
  //  throw std::runtime_error("If provided, \"header/frame_id\" must be \"darpa\".");
  grid.msg.header.frame_id = "darpa";

  // Sanity check on data buffer size
  if ((grid.msg.info.width * grid.msg.info.height) != grid.msg.data.size())
    throw std::runtime_error(
        "Data buffer size (" + std::to_string(grid.msg.data.size()) +
        ") does not match width*height (" + std::to_string(grid.msg.info.width * grid.msg.info.height) +
        ").");
}

// --------------------------------------------------
// Cloud unpacking
// --------------------------------------------------

template <typename T>
void transform_cloud(sensor_msgs::PointCloud2& cloud, const geometry_msgs::Pose& p)
{
  // Translation * Quaternion -> Isometry Transform
  auto t = Eigen::Translation<T,3>(p.position.x, p.position.y, p.position.z) *
    Eigen::Quaternion<T>(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z);

  // Iterator over point cloud
  sensor_msgs::PointCloud2Iterator<T> x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<T> y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<T> z(cloud, "z");
 
  for(; x != x.end(); ++x, ++y, ++z) {
    Eigen::Matrix<T,3,1> point = t * Eigen::Matrix<T,3,1>(*x, *y, *z);
    *x = point.x(); *y = point.y(); *z = point.z();
  }
}

template <typename MsgType>
void unpack_cloud(const MsgType& msg, MapUpdateCloud& cloud)
{
  // ------------------------------
  // Default values
  // ------------------------------
  cloud.msg.header.stamp = ros::Time::now();
  cloud.msg.header.frame_id = "darpa";

  // ------------------------------
  // Unpack msg
  // ------------------------------
  unpack(msg, cloud);

  // ------------------------------
  // Validation
  // ------------------------------
  if (cloud.msg.header.frame_id != "darpa") {
    std::cout << "Got " << cloud.msg.header.frame_id << " instead of darpa frame" << std::endl;
  }


//  if (cloud.msg.header.frame_id != "darpa")
//    throw std::runtime_error("If provided, \"header/frame_id\" must be \"darpa\".");
  cloud.msg.header.frame_id = "darpa";


  // Check point field validity (raises exception for invalid field)
  auto point_field_check = [&cloud](const std::string& name) -> void
  {
    // Ensure field exists
    int index = sensor_msgs::getPointCloud2FieldIndex(cloud.msg, name);
    if (index < 0)
      throw UnprocessableEntity(
          "PointCloud2 must have '" + name + "' point field.");
    // Ensure field has proper type
    uint8_t datatype = cloud.msg.fields[index].datatype;
    if ((datatype != sensor_msgs::PointField::FLOAT32) &&
        (datatype != sensor_msgs::PointField::FLOAT64))
      throw UnprocessableEntity(
          "PointCloud2 point field '" + name + "' must have float32 or float64 datatype.");
  };
  point_field_check("x");
  point_field_check("y");
  point_field_check("z");
  int index_x = sensor_msgs::getPointCloud2FieldIndex(cloud.msg, "x");
  int index_y = sensor_msgs::getPointCloud2FieldIndex(cloud.msg, "y");
  int index_z = sensor_msgs::getPointCloud2FieldIndex(cloud.msg, "z");
  if (cloud.msg.fields[index_x].datatype != cloud.msg.fields[index_y].datatype)
    throw UnprocessableEntity(
        "PointCloud2 point fields 'x' and 'y' datatypes do not match.");
  if (cloud.msg.fields[index_x].datatype != cloud.msg.fields[index_z].datatype)
    throw UnprocessableEntity(
        "PointCloud2 point fields 'x' and 'z' datatypes do not match.");

  // ------------------------------
  // Transformation
  // ------------------------------
  MapUpdateCloudOrigin origin;
  origin.msg.origin.orientation.w = std::numeric_limits<double>::infinity();
  unpack(msg, origin);
  if (origin.msg.origin.orientation.w != std::numeric_limits<double>::infinity())
  {
    uint8_t datatype = cloud.msg.fields[index_x].datatype;
    if (datatype == sensor_msgs::PointField::FLOAT32)
      transform_cloud<float>(cloud.msg, origin.msg.origin);
    else if (datatype == sensor_msgs::PointField::FLOAT64)
      transform_cloud<double>(cloud.msg, origin.msg.origin);
  }

  // ------------------------------
  // Fill in invariances
  // ------------------------------
  cloud.msg.height = 1;
  cloud.msg.width = (cloud.msg.data.size() / cloud.msg.point_step);
  cloud.msg.row_step = cloud.msg.data.size();
  cloud.msg.is_dense = true;
}

// ======================================================================
// Main application
// ======================================================================

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mapping_server");
  ros::NodeHandle nh, pnh("~");

  // ------------------------------------------------------------
  // Application configuration
  // ------------------------------------------------------------

  // Port to listen on for HTTP side
  int port;
  pnh.param("port", port, 11702);

  // Authentication token to require
  std::string token;
  pnh.param("token", token, std::string());

  if (token.empty())
  {
    ROS_FATAL("No authentication token specified");
    return -1;
  }
  if (token.size() != 16)
  {
    ROS_FATAL_STREAM("Authentication token must be 16 characters! Invalid: " << token);
    return -1;
  }
  token = "Bearer " + token;

  // Logging level
  restinio::log_level_t level;
  std::string level_string;
  pnh.param("level", level_string, std::string("info"));
  std::transform(level_string.begin(), level_string.end(), level_string.begin(), ::tolower);
  if ((level_string == "trace") || (level_string == "debug"))
    level = restinio::TRACE;
  else if (level_string == "info")
    level = restinio::INFO;
  else if (level_string == "warn")
    level = restinio::WARN;
  else if (level_string == "error")
    level = restinio::ERROR;
  else
  {
    ROS_FATAL_STREAM("Unknown log level: " << level_string);
    return -1;
  }

  // ------------------------------------------------------------
  // Map and telemetry ROS publishers
  // ------------------------------------------------------------

  // Robot positions
  auto poses_pub = nh.advertise<geometry_msgs::PoseArray>(
      "poses", 1, true);
  auto tf_pub = nh.advertise<tf2_msgs::TFMessage>(
      "/tf", 1, true);
  auto markers_pub = nh.advertise<visualization_msgs::MarkerArray>(
      "markers", 1, true);

  // 2D representations
  auto grid_pub = nh.advertise<nav_msgs::OccupancyGrid>(
      "grid", 1, true);
  // 3D representations
  auto cloud_pub = nh.advertise<sensor_msgs::PointCloud2>(
      "cloud", 1, true);

  // ------------------------------------------------------------
  // State
  // ------------------------------------------------------------

  size_t last_markers_size = 0;
  size_t last_markers_size2 = 0;
  ros::Time last_cloud_time(0.0);

  // ------------------------------------------------------------
  // Map and telemetry content handlers
  // ------------------------------------------------------------

  // Read map message and publish
  auto map_update = [&](auto& content)
  {
    // Unpack our map type information
    MapUpdate type;
    unpack(content, type);
    // Choose appropriate type to unpack
    if (type.type == "OccupancyGrid")
    {
      MapUpdateGrid grid;
      unpack_grid(content, grid);
      ROS_INFO_STREAM("  -> Received grid message with stamp " << grid.msg.header.stamp << " and size " << grid.msg.data.size());
      grid_pub.publish(grid.msg);
    }
    else if (type.type == "PointCloud2")
    {
      MapUpdateCloud cloud;
      unpack_cloud(content, cloud);
      ROS_INFO_STREAM("  -> Received cloud message with stamp " << cloud.msg.header.stamp << " and size " << cloud.msg.data.size());
      cloud_pub.publish(cloud.msg);
      // Check time between messages
      ros::Time curr_time = ros::Time::now();
      if((curr_time - last_cloud_time).toSec() < 1) {
            throw TooManyRequests("Cloud updates too frequent. Reduce to <1Hz.");
      }
      last_cloud_time = curr_time;
    }
    else
    {
      throw std::runtime_error("Unknown \"type\" value \"" + type.type + "\"");
    }
  };
  auto marker_update = [&](auto& content)
  {
    visualization_msgs::MarkerArray markers;
    visualization_msgs::MarkerArray markers_to_send;
    unpack(content, markers);
    // Clear out the rest of the labels (in case poses were deleted)
    for (size_t i = markers.markers.size(); i < last_markers_size2; ++i)
    {
      visualization_msgs::Marker marker;
      marker.id = markers.markers.size();
      marker.action = visualization_msgs::Marker::DELETE;
      markers_to_send.markers.push_back(marker);
    }
    // Remember this marker count
    last_markers_size2 = markers.markers.size();
    for (size_t i =0; i < markers.markers.size(); i++) {
      markers_to_send.markers.push_back(markers.markers[i]);
    }
    markers_pub.publish(markers_to_send);
    
  };
  // Read telemetry message and publish it
  auto telemetry_update = [&](auto& content)
  {
    // Initialize pose array message defaults
    geometry_msgs::PoseArray poses;
    poses.header.stamp = ros::Time::now();
    poses.header.frame_id = "darpa";

    // Unpack our pose array message
    unpack(content, poses);
    // Validate the frame_id
    if (poses.header.frame_id != "darpa") {
      std::cout << "Got " << poses.header.frame_id << " instead of darpa frame" << std::endl;
    }
//      throw std::runtime_error("If provided, \"header/frame_id\" must be \"darpa\".");
    poses.header.frame_id = "darpa";


    // Unpack platform names
    TelemetryNames names;
    unpack(content, names);
    if (poses.poses.size() != names.poses.size())
      throw std::runtime_error("Number of poses and number of names don't match.");
    for (size_t i = 0; i < names.poses.size(); ++i)
      if (names.poses[i].name.empty())
        names.poses[i].name = "robot" + std::to_string(i+1);

    // Publish the pose array
    poses_pub.publish(poses);

    // Publish TF messages
    tf2_msgs::TFMessage tf_msg;
    for (size_t i = 0; i < poses.poses.size(); ++i)
    {
      geometry_msgs::TransformStamped tf;
      tf.header = poses.header;
      tf.child_frame_id = names.poses[i].name;
      tf.transform.translation.x = poses.poses[i].position.x;
      tf.transform.translation.y = poses.poses[i].position.y;
      tf.transform.translation.z = poses.poses[i].position.z;
      tf.transform.rotation    = poses.poses[i].orientation;
      tf_msg.transforms.push_back(tf);
    }
    tf_pub.publish(tf_msg);

    // Publish visualization message
    visualization_msgs::MarkerArray markers;
    for (size_t i = 0; i < poses.poses.size(); ++i)
    {
      // Add a pose marker
      {
        visualization_msgs::Marker marker;
        marker.header = poses.header;
        marker.ns = "pose_array_poses";
        marker.id = markers.markers.size();
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::MODIFY;
        marker.pose = poses.poses[i];
        // rectangular shape
        marker.scale.x = 1.;
        marker.scale.y = 0.7;
        marker.scale.z = 0.5;
        // cyan color
        marker.color.r = 0;
        marker.color.g = 1.;
        marker.color.b = 1.;
        marker.color.a = 1.;
        markers.markers.push_back(marker);
      }
      // Add a text label
      {
        visualization_msgs::Marker marker;
        marker.header = poses.header;
        marker.ns = "pose_array_labels";
        marker.id = markers.markers.size();
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::Marker::MODIFY;
        // position a bit above the pose location
        marker.pose.position = poses.poses[i].position;
        marker.pose.position.z += 0.7;
        marker.pose.orientation.w = 1.0;
        // text height
        marker.scale.z = 0.5;
        // white color
        marker.color.r = 1.;
        marker.color.g = 1.;
        marker.color.b = 1.;
        marker.color.a = 1.;
        // platform name
        marker.text = names.poses[i].name;
        markers.markers.push_back(marker);
      }
    }
    // Clear out the rest of the labels (in case poses were deleted)
    for (size_t i = markers.markers.size(); i < last_markers_size; ++i)
    {
      visualization_msgs::Marker marker;
      marker.id = markers.markers.size();
      marker.action = visualization_msgs::Marker::DELETE;
      markers.markers.push_back(marker);
    }
    // Remember this marker count
    last_markers_size = markers.markers.size();
    markers_pub.publish(markers);
  };

  // ------------------------------------------------------------
  // General HTTP request handler
  // ------------------------------------------------------------

  auto handle_request = [&](auto& callback, auto req, auto)
  {
    ROS_INFO_STREAM("Handling request of size " << req->body().size());
#if 0
    for (size_t i = 0; i < req->body().size(); i++) {
      std::cout << req->body()[i];
    }
    std::cout << std::endl;
#endif
    try {
      // ------------------------------
      // Token authentication check
      // ------------------------------
      auto authorization = req->header().get_field(
            restinio::http_field::authorization,
            std::string()
            );
      if (authorization.empty())
        throw Unauthorized("Authorization not specified.");
      if (token.compare(authorization) != 0)
        throw Unauthorized("Incorrect authorization token.");

      // ------------------------------
      // Read content type
      // ------------------------------
      auto content_type = req->header().get_field(
          restinio::http_field::content_type,
          std::string()
          );
      if (content_type.empty())
        throw UnprocessableEntity("Content-Type not specified.");

      // ------------------------------
      // Handle compressed content encoding
      // ------------------------------
      auto content_encoding = req->header().get_field(
          restinio::http_field::content_encoding,
          "identity"
          );
      std::string unencoded;
      const std::string *body = nullptr;
      if (content_encoding == "identity")
      {
        body = &req->body();
      }
      else if ((content_encoding == "gzip") || (content_encoding == "x-gzip"))
      {
        unencoded = restinio::transforms::zlib::gzip_decompress(req->body());
        body = &unencoded;
      }
      else if (content_encoding == "deflate")
      {
        unencoded = restinio::transforms::zlib::deflate_decompress(req->body());
        body = &unencoded;
      }
      else
      {
        throw UnprocessableEntity(
            "Cannot process Content-Encoding: " + content_encoding);
      }

      // ------------------------------
      // Unpack by content type
      // ------------------------------
      if (content_type == "application/json")
      {
        // Parse JSON content
        json content;
        try {
          content = json::parse(*body);
        } catch (json::exception& ex) {
          throw BadRequest(ex.what());
        }
        // Call with data
        callback(content);
      }
      else if (content_type == "application/cbor")
      {
        // Parse CBOR content
        auto content = cbor::decode(*body);
        if (content.is_undefined())
          throw BadRequest("CBOR decoding error");
        // Call with data
        callback(content);
      }
      else
      {
        throw UnprocessableEntity(
            "Unknown Content-Type: " + content_type);
      }

      // ------------------------------
      // All OK
      // ------------------------------
      return response_ok(std::move(req));
    }

    // ----------------------------------------
    // Application error
    // ----------------------------------------
    catch (HTTPError& err)
    {
      return response_error(std::move(req),
          err.status(), err.what());
    }
    // ----------------------------------------
    // Parse/deserialize/logic error
    // ----------------------------------------
    catch (std::exception& err)
    {
      return response_error(std::move(req),
          restinio::status_unprocessable_entity(),
          flatten_nested_exception(err)
          );
    }
    // ----------------------------------------
    // Server (other) error
    // ----------------------------------------
    catch (...)
    {
      return response_error(std::move(req),
          restinio::status_internal_server_error());
    }
  };

  // ------------------------------------------------------------
  // Start ROS and HTTP server
  // ------------------------------------------------------------

  // Startup ROS in its own thread
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Create our express router
  auto router = std::make_unique< router_t >();
  router->http_post(
      "/map/update",
      [&](auto req, auto params)
      {
        return handle_request(map_update,
            std::move(req), std::move(params));
      }
      );
  router->http_post(
      "/state/update",
      [&](auto req, auto params)
      {
        return handle_request(telemetry_update,
            std::move(req), std::move(params));
      }
      );
  router->http_post(
      "/markers/update",
      [&](auto req, auto params)
      {
        return handle_request(marker_update,
            std::move(req), std::move(params));
      }
      );


  // Startup HTTP server in this thread
  using traits_t =
    restinio::traits_t<
      restinio::asio_timer_manager_t,
      restinio::level_logger_t,
      router_t >;

  restinio::run(
      restinio::on_this_thread<traits_t>()
      .port(port)
      .address("0.0.0.0")
      .logger(level)
      .request_handler( std::move(router) )
      );

  return 0;
}

