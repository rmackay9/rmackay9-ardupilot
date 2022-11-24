# EKF Automatic Source Switching (GPS vs ExtNav)
Switches between AHRS/EKF sources based on the pilot's source selection switch or using an automatic source selection algorithm.
This allows vehicles to move between GPS and Non-GPS environments

# INSTRUCTIONS

Set SCR_ENABLE = 1 to enable scripting on the flight controller
Install this script into the APM/scripts directory and reboot the flight controller

Set EK3_SRC1_xx parameters to use GPS, Baro, Compass, etc (primary)

Set EK3_SRC2_xx parameters to use External Nav (secondary)

Set EK3_SRC3_xx parameters can be left at "None" or to the same as either EK3_SRC1_xxx or EK3_SRC2_xxx to avoid an EKF failsafe if the pilot accidentally selects SRC3

Set RCx_OPTION = 90 (EKF Pos Source) to allow the pilot to directly select the source (low=GPS, middle=ExtNav, high=None)

Set RCx_OPTION = 300 (Scripting1).  If the pilot pulls this switch high, the source will be automatically selected using these thresholds:

- ESRC_GPS_THRESH holds the threshold for GPS horizontal speed accuracy.  Higher values lead to GPS being used more easily
- ESRC_ENAV_THRESH holds the threshold for ExternalNav vertical speed innovation.  Higher values lead to ExternalNav being used more easily
- If both GPS and ExternalNav are working well (e.g. accuracy and innovations are below thresholds), ExtNav will be used
