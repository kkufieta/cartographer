#!/bin/sh
set -o errexit
set -o verbose


VERSION="1.13.0"
CONFIGURATION_DIRECTORY="../configuration_files"
CONFIGURATION_BASENAME="viam_rplidar.lua"
DATA_DIRECTORY="$HOME/data_slam_1"
OUTPUT_DIRECTORY="pictures1"

cd build
rm -rf ${OUTPUT_DIRECTORY}
mkdir ${OUTPUT_DIRECTORY}
./viam_carto_main  \
    -configuration_directory=${CONFIGURATION_DIRECTORY}  \
    -configuration_basename=${CONFIGURATION_BASENAME}  \
    -data_directory=${DATA_DIRECTORY} \
    -output_directory=${OUTPUT_DIRECTORY}
