# EAMS Robotics wind turbine mount control script

EAMS Robotics wind turbine gimbal mount control script

How to use
    Connect AI camera UART to one of the autopilot's serial ports
    Set SERIALx_PROTOCOL = 28 (Scripting) where "x" corresponds to the serial port connected to the AI camera
    Set SCR_ENABLE = 1 to enable scripting and reboot the autopilot
    Copy this script to the autopilot's SD card in the APM/scripts directory and reboot the autopilot
    Adjust EATB_RATE_P and I to adjust the control response to the camera input
    Set EATB_RATE_MAX to the maximum rate in deg/sec that you want the gimbal to rotate
    Set EATB_CONFIG_MIN to the minimum usable confidence from the camera
    Set EATB_DEBUG = 1 to see debug output on the GCS messages tab.  Set to "2" to use test input messages (e.g. fake the camera input)
