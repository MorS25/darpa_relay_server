# Mapping Relay

A ROS package that automatically relays common ROS mapping/telemetry messages directly to the DARPA Mapping Server.  The mapping\_relay node translates ROS messagse into the HTTP requests specified by the official SubT Interface Control Document (ICD), and then transmits them to the DARPA Mapping Server.

This package is meant to make integration between ROS-based systems and the SubT Mapping Server as simple as possible.

## Building

The subt\_mapping directory is configured as a catkin workspace.  To build the mapping\_relay, simply run catkin\_make in the subt\_mapping directory.  You may need to install dependant packages using rosdep or manual installation.  The required dependencies can be found in either the package.xml or CMakeLists.txt files of the mapping\_relay package.

## Running

The mapping\_relay can simply be run as a node in your ROS system.  Use rosrun mapping\_relay mapping\_relay.py to start the node.

## Parameters
The mapping\_relay node has several parameters that may be configured:

* `token`: REQUIRED The team's private token for communicating with the Mapping Server
* `robot_names`: REQUIRED The team's robot names for labeling telemetry data
* `map_url`: OPTIONAL The URL for posting map update HTTP requests to. Default: http://10.100.2.201:8000/map/update
* `state_url`: OPTIONAL The URL for posting state update HTTP requests to. Default: http://10.100.2.201:8000/state/update
* `marker_url`: OPTIONAL The URL for posting marker HTTP requests to. Default: http://10.100.2.201:8000/markers/update
* `compression`: OPTIONAL The compression type to use for data in the HTTP requests. Default: gzip

## Subscribed Topics

The mapping\_relay subscribes to several topics for reading occupancy grids, pointclouds, pose arrays or poses, and marker arrays.  The default topic names are listed below; however teams should remap these topic names as needed.  See the example mapping\_relay.launch file included in the package.

* `grid`: Topic that is publishing nav\_msgs/OccupancyGrid.msgs OR `grid/<ROBOT_NAME>`: Topic[s] that are publishing nav\_msgs/OccupancyGrid.msgs
* `cloud`: Topic that is publishing sensor\_msgs/PointCloud2.msgs OR `cloud/<ROBOT_NAME>`: Topic[s] that are publishing sensor\_msgs/PointCloud2.msgs
* `poses`: Topic that is publishing geometry\_msgs/PoseArray.msgs OR `poses/<ROBOT_NAME>`: Topic[s] that are publishing geometry\_msgs/PoseStamped.msgs
* `marker_array`: Topic that is publishing visualization\_msgs/MarkerArray.msgs

## Caveats

This relay node does not correctly deal with the coordinate frames as required by the ICD, Section 6.1.  In particular, map messages have their `frame_id` field simply overwritten to the required value of `"darpa"` to pass checks on the server-side.

## Test Publishing

The python scripts in the `test` folder can be used to publish randomized point cloud, grid, telemetry, and marker ROS messages. 

The SubT logo is also provided as a test point cloud which can be published using `roslaunch mapping\_relay test\_logo.launch`. 
