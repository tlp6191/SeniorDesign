#!/bin/bash

echo "Meshing $1 $2"
meshlabserver -i $1 -o $2 -s DelaunayTriangulation.mlx
