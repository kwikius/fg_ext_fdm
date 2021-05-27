#!/bin/bash
export QUAN_ROOT=/home/andy/cpp/projects/quan-trunk
if [ $# -eq  0 ]; then
   make
elif [ $# -eq 1 ]; then
   make $1
else
   echo "invalid args"
fi
