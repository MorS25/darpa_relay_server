#!/usr/bin/env bash

# Make sure the mapping_server_bags directory is created on the host for binding.
mkdir -p /home/$USER/mapping_server_bags

# Needed to enable GUI + OpenGL
XAUTH=/tmp/.docker.xauth
if [ ! -f $XAUTH ]
then
    xauth_list=$(xauth nlist :0 | sed -e 's/^..../ffff/')
    if [ ! -z "$xauth_list" ]
    then
        echo $xauth_list | xauth -f $XAUTH nmerge -
    else
        touch $XAUTH
    fi
    chmod a+r $XAUTH
fi

docker run -it --rm \
  -e DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  -e XAUTHORITY=$XAUTH \
  -v "$XAUTH:$XAUTH" \
  --runtime=nvidia \
  -p 8000:8000/tcp \
  --mount type=bind,source="/home/$USER/mapping_server_bags/",target="/home/subt/mapping_server_bags/" \
  mapping_server:0.2
