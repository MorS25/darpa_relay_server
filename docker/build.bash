#!/usr/bin/env bash

docker build -f docker/Dockerfile -t mapping_server:0.2 --build-arg UID=$(id -u) .
