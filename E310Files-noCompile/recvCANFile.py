#This is to run on the Ettus.  It will receive file data from the IHU and or Ragnarok.  There is a place
#to add sending telemetry and receiving other commands.

import can
import random
import time
import serial.tools.list_ports
import os
import sys
from bitmap import BitMap
class CANId:
    # This class represents a CAN mesage ID (or arbitration ID as it is sometimes known).  However
    # this class understands the Golf/LTM meanings of the various fields that we have defined within
    # the 29-bit ID.  You can create the ID by number or field value names as well as changing some fields
    # printing, getting the actually 29-bit number etc.
    #
    #Number/string conversions for source, destination, and type
    
    srcDstNames=["Any","RTIHU_Active","RTIHU_Standby","LIHU","Ragnarok","CIU","Ettus","7",
                   "8","9","10","11","12","13","14","AllBits"]
    typeNames=["NoType","SafeCAN","HealthCAN","ScienceCAN","StatusInfoCAN",
               "HousekeepingCan","EttusCmd","7","WODCAN","SafeAndWODCAN","HealthAndWODCAN"]
    ID1CommandNames=["NotSerial","1","2","3","4","5","6","7","EttusFileData"]
    ID2CommandNames=["Null","EttusEcho","EttusStartSDR","EttusStopSDR","EttusShutdown",
                     "EttusOpenCloseFile","EttusOperationComplete","EttusFileAcknowledge",
                     "EttusTelemUTCTemp","EttusTelemIMU"]
    def __init__(self,value=-1,source="Any",dest="LIHU",thisType="NoType",priority=0x1f,
                 id1="NotSerial",id2="Null",wod=False):
        if(value != -1):
            self.idPriority = (value>>24)&0x1f
            self.idSource = ((value>>20) & 0xf)
            self.idType = ((value>>16) & 0xf)
            self.idDest = ((value>>12) & 0xf)
            self.idId1 = (value>>8) & 0xf
            self.idId2 = value & 0xff;
        else:   
            self.idPriority = priority
            self.idSource = self.srcDstNames.index(source)
            self.idType = self.typeNames.index(thisType)
            self.idDest = self.srcDstNames.index(dest)
            self.idId1 = self.ID1CommandNames.index(id1)
            self.idId2 = self.ID2CommandNames.index(id2)
        
    def __str__(self): #This is for printing an ID
        if(self.idId1 == 0): #If it is not serial
            ID2String = self.ID2CommandNames[self.idId2]
        else:
            ID2String = "{:d}".format(self.idId2)
            #ID2 is an index number.  Just print the number
        str = "ID = {:x} Prio={:d}, source={:s}, dest={:s}, type={:s}, id1={:s},id2={:s}".format(
            self.IDVal(),self.idPriority,self.srcDstNames[self.idSource],self.srcDstNames[self.idDest],
            self.typeNames[self.idType],self.ID1CommandNames[self.idId1],ID2String)

        return str
    
    def IDVal(self): #Get the integer value of the ID to give to "can.Message"
        val = self.idPriority << 24
        val = val | self.idSource << 20
        val = val | self.idType << 16
        val = val | self.idDest << 12
        val = val | self.idId1 << 8
        val = val | self.idId2
        return val
 
    def SetType(self,settype=0,wod=False):
        if(isinstance(settype,int)):
            print("SetType is int:" + str(settype))
            self.idType=settype
        else:
            self.idType = self.typeNames.index(settype)
            if(wod): #WOD boolean only used with strings
                self.idType = self.idType | typeNames.index("WODCAN")
                
    def GetType(self):
        return self.typeNames[self.idType]
    def GetID1(self):
        try:
            return self.ID1CommandNames[self.idId1]
        except:
            return None
        
    def GetID2Numeric(self):
        return self.idId2
    
    def GetID2(self):
        try:
            return self.ID2CommandNames[self.idId2]
        except:
            return None
        
    def SetID2Numeric(self,value):
        self.idId2 = value
        
    def GetSource(self):
        return self.srcDstNames[self.idSource]
        
    def SetDest(self,srcStr):
        self.idDest = srcDstNames.index[srcStr]
        
class GolfCanBus:
    # This class represents the CAN bus on the Golf s/c (and for the most part on LTM as well).
    # It knows how to open the bus (assuming it is a CANable on the system this is running on) and the
    # How to send long messages and files according to the Golf definition.
    thisBusInterface = None
    ackId = CANId(source="Ettus",thisType="EttusCmd",id1="NotSerial",id2="EttusFileAcknowledge")
    def __init__(self,filters=None):
        
        #This is for CANable or some CAN device that uses a serial port
        canDevice = None
        portInfos = serial.tools.list_ports.grep("CAN")
        for pi in portInfos:
            canDevice = pi[0]
            print("Found CANable "+canDevice)
        if(canDevice != None):
            self.thisBusInterface = can.interface.Bus(bustype='slcan', channel=canDevice, bitrate=125000,
                        can_filters=filters)
        else:
            #This one is for a socket type device (i.e. the Ettus)
            self.thisBusInterface = can.interface.Bus(bustype='socketcan_native', channel='can0', bitrate=125000,
                        can_filters=None)

        print("Bus is created")
        
    def SendAck(self,ackData=None):       
        if(ackData == None):
            raise Exception("Ack requested with no data specified")
        msg=can.Message(is_extended_id=True,arbitration_id=self.ackId.IDVal(),
            data=ackData)
        print(msg)                
        self.thisBusInterface.send(msg)

        
    def RecvAndDispatchMessages(self):
        thisFile = None
        while True:
            #print("Wait for message")
            thisMsg = self.ReceiveCanMsg()
            if(thisMsg == None):
                print("No message received")
                #Here we could send telemetry
                continue
            id = CANId(value=thisMsg.arbitration_id)
            #print("Message received with type " + id.GetType() + "=" +hex(id.IDVal()))
            #print("Msg received="+str(thisMsg))
            #print("ID means"+str(id)+"Type is "+id.GetType())
            if(id.GetType() != "EttusCmd"):
                continue
            if(id.GetID1() == "NotSerial"):
                command = id.GetID2()
            else:
                command = id.GetID1()
            #print("Received command "+command)
#             Match command: #match is not available until Python 3.10.  Don't use for now.
            if(command == "EttusShutdown"):
                exit()
            if(command == "EttusOpenCloseFile"):
                if(thisMsg.data[0] == 0): #Open
                    print("Open file #"+ str(thisMsg.data[1]))
                    self.ackId.SetDest = id.GetSource()
                    self.thisFile = File(self,thisMsg.data) #Have to deal with out of order, already open etc
                    self.SendAck(ackData=thisMsg.data)
                elif(thisMsg.data[0] == 1):
                    print("Close file #"+ str(thisMsg.data[1]))
                    self.thisFile.RecvCloseMsg(thisMsg.data)
                else:
                    raise Exception("Unknown argument in file open/close")
            if(command == "EttusFileData"):
                self.thisFile.RecvFileDataMsg(id,thisMsg.data)
        
    
    def ReceiveCanMsg(self):
        try:
            msg = self.thisBusInterface.recv(4) #Wait briefly for an incoming msg.  This is also how often to send an outgoing msg
        except:
            msg = None
        return msg

class File:
    def __init__(self,canBus,msgDat):
        self.fileNum = msgDat[1]
        print("File init with number "+str(self.fileNum))
        self.canBus = canBus
        fileText = "DataFile"+ str(self.fileNum)
        fileName = os.path.join(os.path.expanduser("~"),fileText)
        self.thisFile=open(fileName,"wb")
        self.longMsg = LongCanMsg() #Get the first long CAN message object
        self.currentFileLength = 0; #Total bytes received for file after the last long Can message group
        self.closeFileLength = None;  #Total bytes expected for the file (sent via close message)
        self.finalAckData = None;

        
    def RecvFileDataMsg(self,canId,data):
        if(self.thisFile == None):
            raise Exception("File data coming before file open")
        thisChunkLength = self.longMsg.IncomingCanMsg(canId,data)
        self.currentFileLength += thisChunkLength
        #print("Current file length so far is {:d}".format(self.currentFileLength))
        if(self.currentFileLength == self.closeFileLength): #Close came before last data message, but all done now
            self.thisFile.write(self.longMsg.GetLongData())
            self.canBus.SendAck(ackData=self.finalAckData) #Close has come, final data has come. Send ack.
            self.thisfile.close()
            self.thisFile = None

        elif(self.longMsg.IsFull()):
            self.thisFile.write(self.longMsg.GetLongData())
            self.longMsg = LongCanMsg() #Start a new long message.  Supposedly the old longMsg gets garbage collected
            self.canBus.SendAck(ackData=[2,self.fileNum]) #Ack the long message

    def RecvCloseMsg(self,data):
        if(self.fileNum != data[1]):
            raise Exception('File open is #{:d} but closing #{:d}'.format(self.fileNum,data[0]))
        closeLength = data[2] + (data[3]*256) + (data[4]*65536)
        print("Received close specing length={:d} and my current len is {:d}".format(closeLength,
                self.currentFileLength))
        if(closeLength == self.currentFileLength):
            self.canBus.SendAck(ackData=data) #Data for the ack is the same as for the close
            if (self.longMsg.HasData()):
                self.thisFile.write(self.longMsg.GetLongData())
            if(self.thisFile != None):
                self.thisFile.close()
                self.thisFile = None
        else:
            #Here we got a close out of order before all the data has arrived.  Save the message data for
            #from the close to send with the ack
            self.finalAckData = data
        exit() #Temporary

class LongCanMsg:
    #BitMap Functions
    #BitMap(maxnum): construct a BitMap object with maxnum bits
    #set(pos): set the bit at position pos to 1
    #reset(pos): reset the bit at position pos to 0
    #flip(pos): flip the bit at position pos
    #count(): return the number of 1s
    #size(): return the size of the BitMap
    #test(pos): check if bit at position pos has been set to 1
    #any(): check if any bit in the BitMap has been set to 1
    #none(): check if none of the bits in the BitMap has been set to 1
    #all(): check if all bits in the BitMap has been set to 1
    #nonzero(): return indexes of all non-zero bits
    #tostring(): convert a BitMap object to 0 and 1 string
    #fromstring(bitstring): create a BitMap object from 0 and 1 string

    def __init__(self):
        self.longData = bytearray(255*8)  # 255 messages (chunks), each of which can have 8 bytes
        self.totalLength = 0
        self.msgBitMap = BitMap(255) # Keep track of which message received. It starts out initted to 0
        self.maxChunkRcvd = -1
        self.endChunkRcvd=False
        self.longMessageDone=False

    def GetLength(self):
        return self.totalLength
    
    def IsFull(self):
        return self.longMessageDone
    
    def HasData(self):
        return bool(self.totalLength != 0)
    
    def IncomingCanMsg(self,canId,data):
        #We assume we were only called if the Id was already identified as data for us
        chunkNum = canId.GetID2Numeric() #With long data, Id2 contains the index number which we call a chunk
        if(self.msgBitMap.test(chunkNum)):
            raise Exception('Duplicate Long File Chunk: ' + str(chunkNum))
        byteNum = chunkNum*8
        chunkSize = len(data) #This should be 8 unless it is the last one
        self.totalLength += chunkSize
        self.msgBitMap.set(chunkNum)
        if(chunkNum > self.maxChunkRcvd):
            self.maxChunkRcvd = chunkNum
            if(self.endChunkRcvd):
                raise Exception('Chunk number {:d} greater than final chunk ({:d}) '.format(chunkNum,
                                                                                self.maxChunkRcvd))                
        self.longData[byteNum:byteNum+chunkSize] = data
        if((chunkSize<8) or (chunkNum==255)):
            print("Setting end received to true")
            self.endChunkRcvd=True
        self.longMessageDone = self.endChunkRcvd and self.NoMissingMessages()
        return chunkSize
    
    def NoMissingMessages(self):
        print("No missing:  maxChunkRcvd={:d}, bit map count={:d}".format(self.maxChunkRcvd,
                                                                          self.msgBitMap.count()))
        return ((self.maxChunkRcvd+1) == self.msgBitMap.count()) #+1 to fix fencepost.  Index vs count
    
    def GetLongData(self):
        return self.longData[0:self.totalLength]

################################################
#  Here is where the program starts running
################################################
id=CANId(dest="Ettus",priority=0)
idMask=CANId(dest="AllBits",priority=0)
canFilters=[{"can_id":id.IDVal(),"can_mask":idMask.IDVal(),"extended":True}]
print("ID="+hex(id.IDVal()))
print("Mask="+hex(idMask.IDVal()))
#thisBus = GolfCanBus(filters=canFilters)
thisBus = GolfCanBus(filters=None)
thisBus.RecvAndDispatchMessages()

