#!/bin/bash

echo "$1"
./statistical_removal $1 500 .5
mv "inliers_$1" "$2"
