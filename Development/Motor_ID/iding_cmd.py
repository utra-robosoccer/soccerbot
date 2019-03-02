# import serial.tools.list_ports
import serial
import argparse
import serial.tools.list_ports

'''
VALID ID INPUT: _ _ _ _ *4 characters in length*
  - first two characters: AX, M1, M2
        - AX : IDing AX12As
        - M1:  IDing MX28s at 57600 bps
        - M2:  IDing MX28s at 1 Mbps
  - last two characters: 01, 02, ... , 12, 13, ..
  - in the command line, write without quotation marks
'''

# parse command line 
parser = argparse.ArgumentParser(description='Process command lines')
parser.add_argument('motorType', action = 'store', help='stores a string with 2 characters, where the first two characters (must be capital letters) represent a motor type: AX(IDing AX12As), M1(IDing MX28s at 57600 bps), M2(IDing MX28s at 1 Mbps)')
parser.add_argument('idNumber', action = 'store', help='stores a string with the motor number: (01,02,..,13,14...)')

# port information
ports = list(serial.tools.list_ports.comports())
if (len(ports)==0):
    raise Exception('No ports detected')
for port in ports:
#   Takes in the first connected port
    dev_name = port.device

port = dev_name
baud = 115200
  
ser = serial.Serial(port, baud, timeout=1)
if ser.isOpen():
     print(ser.name + ' is open...\n')

id_string = None

args = parser.parse_args()
motorType = args.motorType
idNumber = args.idNumber
id_string = motorType + idNumber

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
        
        