#!/bin/bash

terminator -x bash -c "source ~/tensorflow/bin/activate; rosrun gp_lio lpdnet_node.py; exec bash" &

terminator -x bash -c "roslaunch gp_lio estimator_node_HACTL.launch; exec bash"
