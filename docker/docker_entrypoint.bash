#!/usr/bin/env bash

# Docker WORKDIR is /home/$USERNAME/subt_ws

# Source the package's install script
source install/setup.bash

# Launch the mapping server.
exec roslaunch mapping_server mapping_server.launch
