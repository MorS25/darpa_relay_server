# Mapping Server

This repository contains several packages implementing various aspects of the DARPA SubT Mapping Server:

* **mapping_server**: A ROS catkin package implementing the Mapping Server
* **mapping_relay**: A ROS catkin package containing a relay utility for transmitting common ROS mapping/telemetry messages directly to the Mapping Server via HTTP.
* **docker**: A directory containing docker files for building and running the Mapping Server as a standalone docker container.
* **cbor**: A CBOR library that is included as a dependency of the Mapping Server.
* **json**: A JSON library that is included as a dependency of the Mapping Server.

Further documentation for the mapping\_server and mapping\_relay can be found in README files within the respective package directories.
