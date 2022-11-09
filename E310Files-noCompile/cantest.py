#!/usr/bin/python3
#
# simple_rx_test.py
# 
# This is simple CAN receive python program. All messages received are printed out on screen.
# For use with PiCAN boards on the Raspberry Pi
# http://skpang.co.uk/catalog/pican2-canbus-board-for-raspberry-pi-2-p-1475.html
#
# Make sure Python-CAN is installed first http://skpang.co.uk/blog/archives/1220
#
# 01-02-16 SK Pang
#
#
#

import can
import time
import os
import ctypes

#define ID2 command values
CANId2EttusEcho = 1
CANId2EttusStartSDR = 2
CANId2EttusStopSDR = 3
CANId2EttusShutdown = 4
CANId2OpenCloseFile = 5
CANId2EttusOperationComplete = 6 # This is to indicate startup or shutdown complete
CANId2EttusFileAcknowledge = 7   #Acknowledge a file open, close, or data acceptance
CANId2EttusTelemUTCTemp = 8
CANId2EttusTelemIMU = 9
CANIdTelem = 0x1f620000
CANIdCmdResponse = 0x1f660000




def CANCmdEcho(bus,inMessage):
    print("Echo")
    outMessage = inMessage
    outMessage.arbitration_id = CANIdCmdResponse + CANId2EttusEcho
    bus.send(outMessage,1.0)
    print("Done sending echo")

def CANCmdStartSDR(bus,inMessage):
    print("Start SDR")
    
def CANCmdStopSDR(bus,inMessage):
    print("Stop SDR")

def CANCmdShutdown(bus,inMessage):
    print("Shutdown cleanly")
    os.system("shutdown now")
    
def CANCmdOpenCloseFile(bus,inMessage):
    print("OpenClose File")


print('\n\rCAN Rx test')
print('Bring up CAN0....')
try:
        os.system("ip link set can0 type can bitrate 125000")
	time.sleep(0.1)
	os.system("ip link set up can0")
except:
	e = sys.exc_info()[0]
	print("Error is %d" % e)
try:
	bus = can.interface.Bus(channel='can0', bustype='socketcan_native')
except OSError:
	print('Cannot open can socket')
	exit()
	
print('Ready')
# Set filter to receive only messages with ettus as the destination
bus.set_filters(filters=[{"can_id": 0x00006000, "can_mask": 0x0000f000}])

#
# Set up standard messages to send
#
msgUTCandTemp = can.Message(arbitration_id=CANIdTelem+CANId2EttusTelemUTCTemp,dlc=8)
msgIMU = can.Message(arbitration_id=CANIdTelem+CANId2EttusTelemIMU,dlc=8)
msgOpComplete = can.Message(arbitration_id=CANIdCmdResponse+CANId2EttusOperationComplete,dlc=8)


#
# Here is a list of the function to call when we get the specified message in Id2
#
function_dict = {
	CANId2EttusEcho : CANCmdEcho,
	CANId2EttusStartSDR : CANCmdStartSDR,
	CANId2EttusStopSDR: CANCmdStopSDR,
	CANId2EttusShutdown: CANCmdShutdown
	}

try:
	while True:
		print("Listening")
		message = bus.recv(timeout=4.0)	# Wait until a message is received.
		if (message == None):
		    print("Timed out")
		    msgIMU.data = bytearray([0,1,2,3,4,5,6,7])
		    bus.send(msgIMU,1.0)
		else:
		    c = '{0:f} {1:x} {2:x} '.format(message.timestamp, message.arbitration_id, message.dlc)
		    s=''
		    for i in range(message.dlc ):
			s +=  '{0:x} '.format(message.data[i])
			
		    print(' {}'.format(c+s))
		    function_dict[message.arbitration_id & 0xff](bus,message) #Execute the function matching the bottom 8 bits of the ID
		
	
except KeyboardInterrupt:
	#Catch keyboard interrupt
#	os.system("sudo /sbin/ip link set can0 down")
	print('\n\rKeyboard interrupt')	

