# import serial.tools.list_ports
import serial
import argparse
import serial.tools.list_ports

'''
VALID ID INPUT: _ _ _ _ *4 characters in length*
  - first two characters: AX, M1, M2
  - last two characters: 01, 02, ... , 12, 13, ..
  - in the command line, write without quotation marks
'''

# port information
port = 'COM6'
baud = 115200
  
ser = serial.Serial(port, baud, timeout=1)
if ser.isOpen():
     print(ser.name + ' is open...\n')

# parse command line 
parser = argparse.ArgumentParser(description='Process command lines')
parser.add_argument('id_string', action = 'store', help='stores a string with 4 characters, where the first two characters (must be capital letters) represent a motor type (AX, M1, M2) and the last two characters represent the motor number (01,02,..,13,14...)')

id_string = None

# parser.add_argument('--id', action='store')
args = parser.parse_args()
id_string = args.id_string
if(len(id_string)!=4):
    print("ERROR: The motor id argument must be 4 characters")
    exit()

print("Sending ID string...", )
ser.write(id_string.encode('ascii'))
out = ser.readline()
print('Receiving...', str(out))
ser.close()

exit()

# exit()
        
        