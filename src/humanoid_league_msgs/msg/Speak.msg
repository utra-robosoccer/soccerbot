# This message is used to activate the audio output of the robot
# This can be used for debug proposed but also for natural language team communication

# The text will only be outputed if "filename" is empty
string text

uint8 LOW_PRIORITY=0
uint8 MID_PRIORITY=1
uint8 HIGH_PRIORITY=2
uint8 priority

# If a file should be read, the path has to be specified here. Otherwise this string should be null
string filename