#!/bin/bash
source install/setup.bash

receive_port=2020
send_port=2019

if [ $# -gt 0 ]
then
    receive_port=$(($receive_port + $1 * 2))
    send_port=$(($send_port + $1 * 2))
fi
echo "r: $receive_port, s: $send_port"
micrortps_agent -t UDP -r $receive_port -s $send_port
