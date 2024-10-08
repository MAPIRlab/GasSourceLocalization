#!/bin/bash

function run()
{
    mkdir $1
    ros2 launch pmfs_env gaden_preproc_launch.py floor_height:=$1
}

height="0.0"
increment=$(echo 0.08 | bc)
for i in {1..12}
do
    run $(printf "%0.2f" $height)
    height=$(echo $height+$increment | bc) 
done