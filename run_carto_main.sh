#!/bin/sh
set -o errexit
set -o verbose


# ---- Edit based on your needs:
DATE="02242022"

MAPPING="false"
LOCALIZATION="false"
UPDATE="true"

MAPPING_DATA_DIRECTORY="$HOME/rplidar/data_large"
LOCALIZATION_DATA_DIRECTORY="$HOME/rplidar/data_small"
# UPDATE_DATA_DIRECTORY="$HOME/rplidar/data_hexa_022422"
UPDATE_DATA_DIRECTORY="$HOME/rplidar/data_printer_room"

OUTPUT_DIRECTORY="pics_${DATE}_new_data_hexa"
MAP_OUTPUT_NAME="map_${DATE}.pbstream"

PICTURE_PRINT_INTERVAL="300"
# ----

CONFIGURATION_DIRECTORY="../configuration_files"
CONFIGURATION_MAPPING_BASENAME="viam_mapping.lua"
CONFIGURATION_LOCALIZATION_BASENAME="viam_localization.lua"
CONFIGURATION_UPDATE_BASENAME="viam_update_map.lua"

cd build
rm -rf ${OUTPUT_DIRECTORY}
mkdir ${OUTPUT_DIRECTORY}
./viam_carto_main  \
    -configuration_directory=${CONFIGURATION_DIRECTORY}  \
    -configuration_mapping_basename=${CONFIGURATION_MAPPING_BASENAME}  \
    -configuration_localization_basename=${CONFIGURATION_LOCALIZATION_BASENAME}  \
    -configuration_update_basename=${CONFIGURATION_UPDATE_BASENAME}  \
    -mapping=${MAPPING}  \
    -localization=${LOCALIZATION}  \
    -update=${UPDATE}  \
    -mapping_data_directory=${MAPPING_DATA_DIRECTORY} \
    -localization_data_directory=${LOCALIZATION_DATA_DIRECTORY} \
    -update_data_directory=${UPDATE_DATA_DIRECTORY} \
    -output_directory=${OUTPUT_DIRECTORY} \
    -map_output_name=${MAP_OUTPUT_NAME} \
    -picture_print_interval=${PICTURE_PRINT_INTERVAL}
