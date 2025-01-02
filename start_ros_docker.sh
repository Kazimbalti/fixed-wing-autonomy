#!/bin/bash
docker run \
    -it \
    --mount type=bind,source="$(pwd)/workspace",target=/opt/autonomy \
    $pathmount \
    --network=host \
    ros2_autonomy

