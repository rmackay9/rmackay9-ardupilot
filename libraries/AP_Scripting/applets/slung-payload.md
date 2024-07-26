# Slung Payload

This script reduces the oscillation of a slung payload that is capable of sending its position and velocity to the main vehicle

# Parameters

SLUP_POS_P : Oscillation controller position P gain

# How To Use

1. copy this script to the autopilot's "scripts" directory
2. within the "scripts" directory create a "modules" directory
3. copy the MAVLink/mavlink_msgs_xxx files to the "scripts" directory

# How It Works

The script's algorithm is implemented as follows

1. Consume GLOBAL_POSITION_INT messages from the payload
2. Calculate the payload's position vs the vehicle position
3. Use a P controller to move the vehicle towards the payload to reduce oscillation
