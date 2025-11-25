#!/bin/sh

# Source the default startup script
. ${PX4_SOURCE_DIR}/build/px4_sitl_default/etc/init.d-posix/rcS

# Load custom parameters
param set EKF2_EV_CTRL 15
param set EKF2_HGT_REF 3
param set EKF2_GPS_CTRL 0
param set COM_ARM_WO_GPS 1
param set COM_RCL_EXCEPT 4

# Restart EKF2 to apply new parameters (specifically GPS disable)
param set EKF2_GPS_CHECK 0
param set COM_POS_FS_EPH 0
param set COM_POS_FS_EPV 0
