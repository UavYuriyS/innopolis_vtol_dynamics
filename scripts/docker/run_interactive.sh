#!/bin/bash
cd "$(dirname "$0")"
source config.sh
sudo docker container run --privileged -v /dev/bus/usb:/dev/bus/usb \
                          -it                                       \
                          --net=host                                \
                          $DOCKERHUB_REPOSITOTY:$TAG_NAME           \
                          /bin/bash