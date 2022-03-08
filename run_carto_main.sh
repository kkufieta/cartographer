#!/bin/sh
set -o errexit
set -o verbose


# ---- Edit based on your needs:
DATE="02242022"

OUTPUT_DIRECTORY="pics_${DATE}_new_data_hexa"

# Test 1
MAPPING_DATA_DIRECTORY="$HOME/rplidar/data_first_success"
MAP_OUTPUT_NAME="map_input_${DATE}.pbstream"
#MAP_INPUT_NAME="map_input_${DATE}.pbstream"

# Test 2
MAPPING_DATA_DIRECTORY="$HOME/rplidar/data_printer_room"
# MAP_OUTPUT_NAME="map_${DATE}.pbstream"
MAP_INPUT_NAME="map_input_${DATE}.pbstream"

# Test 3
MAPPING_DATA_DIRECTORY="$HOME/rplidar/data_printer_room"
MAP_OUTPUT_NAME="map_output_${DATE}.pbstream"
MAP_INPUT_NAME="map_input_${DATE}.pbstream"

PICTURE_PRINT_INTERVAL="300"
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
    -map_output_name=${MAP_INPUT_NAME} \
    -map_output_name=${MAP_OUTPUT_NAME} \
    -picture_print_interval=${PICTURE_PRINT_INTERVAL}
