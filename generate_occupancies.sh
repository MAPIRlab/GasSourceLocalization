#!/bin/bash

function run()
{
    mkdir $1
    ros2 launch pmfs_env gaden_preproc_launch.py floor_height:=$1
}

run 0.05
run 0.15
run 0.20
run 0.25
run 0.30
run 0.35
run 0.40
run 0.45
run 0.50
run 0.55
run 0.65
run 0.70
run 0.75
run 0.80