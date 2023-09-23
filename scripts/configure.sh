#!/bin/bash
SCRIPT_NAME=$(basename $BASH_SOURCE)
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
REPOSITORY_DIR="$(dirname "$SCRIPT_DIR")"
RED='\033[0;31m'
YELLOW='\033[0;33m'
NC='\033[0m' # No Color
print_help() {
   echo "usage: docker.sh COMMAND [OPTIONS]

Automatical configuration:
1. Download firmware
2. Upload firmware to the autopilot
3. Configure parameters"
}

configure_px4_v1_14_cyphal_quadcopter() {
    wget -O $DOWNLOADED_BINARY_PATH $PX4_V1_14_0_BETA_CYPHAL_URL
    autopilot-configurator -v --firmware $DOWNLOADED_BINARY_PATH -f --config \
        ${REPOSITORY_DIR}/configs/px4_v1.14_quadcopter_airframe.yaml \
        ${REPOSITORY_DIR}/configs/px4_cyphal.yaml \
        ${REPOSITORY_DIR}/configs/px4_common.yaml
}

configure_px4_v1_14_cyphal_octorotor() {
    wget -O $DOWNLOADED_BINARY_PATH $PX4_V1_14_0_BETA_CYPHAL_URL
    autopilot-configurator -v --firmware $DOWNLOADED_BINARY_PATH -f --config \
        ${REPOSITORY_DIR}/configs/px4_v1.14_octorotor_airframe.yaml \
        ${REPOSITORY_DIR}/configs/px4_cyphal.yaml \
        ${REPOSITORY_DIR}/configs/px4_common.yaml
}

configure_px4_v1_14_dronecan_quadrotor() {
    wget -O $DOWNLOADED_BINARY_PATH $PX4_V1_14_0_BETA_DRONECAN_URL
    autopilot-configurator -v --firmware $DOWNLOADED_BINARY_PATH -f --config \
        ${REPOSITORY_DIR}/configs/px4_v1.14_quadcopter_airframe.yaml \
        ${REPOSITORY_DIR}/configs/px4_dronecan.yaml \
        ${REPOSITORY_DIR}/configs/px4_common.yaml
}

configure_px4_v1_12_dronecan_vtol() {
    wget -O $DOWNLOADED_BINARY_PATH $PX4_V1_12_1_DRONECAN_URL
    autopilot-configurator -v --firmware $DOWNLOADED_BINARY_PATH -f --config \
        ${REPOSITORY_DIR}/configs/px4_v1.13_inno_vtol_airframe.yaml \
        ${REPOSITORY_DIR}/configs/px4_dronecan.yaml \
        ${REPOSITORY_DIR}/configs/px4_common.yaml
}

if [ -z $1 ]; then
    printf "$RED$SCRIPT_NAME ERROR (line ${LINENO}): Argument is not specified!$NC\n"
    exit 1
fi

# Check autopilot_tools version
autopilot_tools_version=$(pip show autopilot_tools | grep Version)
if [ ! -z "$autopilot_tools_version" ]; then
    IFS=':' read -ra version <<< "$autopilot_tools_version"
    IFS='.' read -ra version <<< ${version[1]}
    if (( ${version[1]} >= 4 )); then
        printf "$SCRIPT_NAME INFO: autopilot-tools: $autopilot_tools_version\n"
    else
        printf "$YELLOW$SCRIPT_NAME WARN: autopilot-tools: $autopilot_tools_version (Consider updating)$NC\n"
    fi
else
    printf "$RED$SCRIPT_NAME ERROR (line ${LINENO}): autopilot-tools is not installed!$NC\n"
    exit 1
fi


DOWNLOADS_DIR=$REPOSITORY_DIR/downloads
DOWNLOADED_BINARY_PATH=$DOWNLOADS_DIR/px4_fmu-v5_latest_downloaded_firmware.px4
PX4_V1_12_1_DRONECAN_URL=https://github.com/ZilantRobotics/PX4-Autopilot/releases/download/v1.12.1_hitl/px4_fmu-v5_default.px4
PX4_V1_14_0_BETA_CYPHAL_URL=https://github.com/ZilantRobotics/PX4-Autopilot/releases/download/v1.14.0-0.3.0-beta1/px4_fmu-v5_cyphal.px4
PX4_V1_14_0_BETA_DRONECAN_URL=https://github.com/ZilantRobotics/PX4-Autopilot/releases/download/v1.14.0-0.3.0-beta1/px4_fmu-v5_default.px4


mkdir -p $DOWNLOADS_DIR
rm -f $DOWNLOADED_BINARY_PATH

if [[ $1 == "cyphal_quadrotor" ]]; then
    configure_px4_v1_14_cyphal_quadcopter
elif [[ $1 == "cyphal_octorotor" ]]; then
    configure_px4_v1_14_cyphal_octorotor
elif [[ $1 == "dronecan_quadrotor" ]]; then
    configure_px4_v1_14_dronecan_quadrotor
elif [[ $1 == "dronecan_vtol" ]]; then
    configure_px4_v1_12_dronecan_vtol
else
    printf "$RED$SCRIPT_NAME ERROR (line ${LINENO}): Unknown argument: '$1' $NC\n"
    exit 1
fi