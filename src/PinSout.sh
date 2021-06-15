#!/bin/sh

export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH
export PATH="/root/anaconda3/envs/envname/bin:$PATH"

echo Starting the PinSout
echo----------------------------------------------------------------------------
echo
echo First argument : Path of PointCloud data
echo Second argument : Value of distance threshold for Plane RANSAC default=0.05
echo Third argument : Value of Epslion for Intersection Function default=0.1
echo ---------------------------------------------------------------------------
echo
echo There are $# arguments to $0: $*
echo First argument : $1
echo Second argument : $2
echo Third argument : $3
echo Here they are again: $@
echo ---------------------------------------------------------------------------
echo
python plyTo3Dmodel.py $1 $2 $3
