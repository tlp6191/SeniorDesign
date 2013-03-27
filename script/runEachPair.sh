#!/bin/bash

echo "Merging $1 $2, storing output in $3"
./pairwise_incremental_registration $1 $2
mv 1.ply $3
