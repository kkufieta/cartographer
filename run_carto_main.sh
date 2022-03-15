#!/bin/sh
set -o errexit
set -o verbose


# ---- Edit based on your needs:
DATE="02242022"


# Test 1
OUTPUT_DIRECTORY="thrid_floor_mobile4_${DATE}_mapping"
DATA_DIRECTORY="$HOME/rplidar/thrid-floor-mobile4"
MAP_OUTPUT_NAME="map_thrid_floor_mobile_${DATE}.pbstream"
#MAP_INPUT_NAME="map_input_${DATE}.pbstream"

# Test
# OUTPUT_DIRECTORY="one_config_test_${DATE}_localization"
# DATA_DIRECTORY="$HOME/rplidar/data_Feb_11_2022_small"
# # MAP_OUTPUT_NAME="map_${DATE}.pbstream"
# MAP_INPUT_NAME="map_input_${DATE}.pbstream"

# Test 3
# OUTPUT_DIRECTORY="one_config_test_${DATE}_updating"
# DATA_DIRECTORY="$HOME/rplidar/data_Feb_25_2022_help_hecto_carpet"
# MAP_OUTPUT_NAME="map_output_${DATE}.pbstream"
# MAP_INPUT_NAME="map_input_${DATE}.pbstream"

PICTURE_PRINT_INTERVAL="10"
# ----

CONFIGURATION_DIRECTORY="../configuration_files"
CONFIGURATION_BASENAME="viam_mapping.lua"

cd build
rm -rf ${OUTPUT_DIRECTORY}
mkdir ${OUTPUT_DIRECTORY}
./viam_carto_main  \
    -configuration_directory=${CONFIGURATION_DIRECTORY}  \
    -configuration_basename=${CONFIGURATION_BASENAME}  \
    -data_directory=${DATA_DIRECTORY} \
    -output_directory=${OUTPUT_DIRECTORY} \
    -map_input_name=${MAP_INPUT_NAME} \
    -map_output_name=${MAP_OUTPUT_NAME} \
    -picture_print_interval=${PICTURE_PRINT_INTERVAL}
