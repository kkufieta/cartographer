#!/bin/sh
set -o errexit
set -o verbose


# ---- Edit based on your needs:
MAPPING="true"
LOCALIZATION="true"

DATA_DIRECTORY="$HOME/rplidar/data_first_success"
# DATA_DIRECTORY="$HOME/rplidar/data"

OUTPUT_DIRECTORY="pic_large_02182022"
MAP_OUTPUT_NAME="map_02182022.pbstream"

PICTURE_PRINT_INTERVAL="500"
# ----

CONFIGURATION_DIRECTORY="../configuration_files"
CONFIGURATION_MAPPING_BASENAME="viam_mapping.lua"
CONFIGURATION_LOCALIZATION_BASENAME="viam_localization.lua"

cd build
rm -rf ${OUTPUT_DIRECTORY}
mkdir ${OUTPUT_DIRECTORY}
./viam_carto_main  \
    -configuration_directory=${CONFIGURATION_DIRECTORY}  \
    -configuration_mapping_basename=${CONFIGURATION_MAPPING_BASENAME}  \
    -configuration_localization_basename=${CONFIGURATION_LOCALIZATION_BASENAME}  \
    -mapping=${MAPPING}  \
    -localization=${LOCALIZATION}  \
    -data_directory=${DATA_DIRECTORY} \
    -output_directory=${OUTPUT_DIRECTORY} \
    -map_output_name=${MAP_OUTPUT_NAME} \
    -picture_print_interval=${PICTURE_PRINT_INTERVAL}
