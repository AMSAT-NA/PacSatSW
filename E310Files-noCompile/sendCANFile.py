import can
import random
import time
import serial.tools.list_ports
import os
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
    def __init__(self,value=-1,source="Any",dest="LIHU",thistype="NoType",priority=0x1f,
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
            self.idType = self.typeNames.index(thistype)
            self.idDest = self.srcDstNames.index(dest)
            self.idId1 = self.ID1CommandNames.index(id1)
            self.idId2 = self.ID2CommandNames.index(id2)
        
    def __str__(self): #This is for printing an ID
        str = "ID = {:x} Prio={:d}, source={:s}, dest={:s}, type={:s}, id1={:s},id2={:s}".format(
        self.IDVal(),self.idPriority,self.srcDstNames[self.idSource],self.srcDstNames[self.idDest],
         self.typeNames[self.idType],self.ID1CommandNames[self.idId1],self.ID2CommandNames[self.idId2])
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
    def GetID2(self):
        try:
            return self.ID2CommandNames[self.idId2]
        except:
            return None
    def GetId2Numeric(self):
            return self. idId2
    def SetID2Numeric(self,value):
        self.idId2 = value
        
        
class GolfCanBus:
    # This class represents the CAN bus on the Golf s/c (and for the most part on LTM as well).
    # It knows how to open the bus (assuming it is a CANable on the system this is running on) and the
    # How to send long messages and files according to the Golf definition.
    
    thisBus = None
    def __init__(self,canDevice=None,filters=None):
        if(canDevice==None):
            portInfos = serial.tools.list_ports.grep("CAN")
            for pi in portInfos:
                canDevice = pi[0]
                print("Opening "+canDevice)
        self.thisBus = can.interface.Bus(bustype='slcan', channel=canDevice, bitrate=125000,
                        can_filters=filters)
        print("Bus is created")
    def WaitForAck(self,fileNum):
        # This method assumes we are sending a file to the Ettus device on Golf and knows how
        # to wait for an acknowledge packet.   It is not clever enough to deal with packets that
        # are NOT the ack packet.
        
        ack = None
        id2 = None
        while(ack==None or id2 != "EttusFileAcknowledge"):
        #print("Fake ack")
        #while(0): #Temporarily ignore the ack for testing
            print("Waiting for ack")
            ack = self.thisBus.recv(2)
            if(ack != None):
                ackId = CANId(value=ack.arbitration_id)
                id2=ackId.GetID2()
                print("Msg received="+ str(ack))
                print("ID2 is" + id2)
        print("Return from WaitForAck")
    def SendFile(self,fileNum,fileName):
        # This method knows how to send a file (usually to the Ettus) according to the defined
        # Golf protocol.
        fileLength = 0
        openCloseData = bytearray([0,fileNum])
        fileOpenCloseID = CANId(dest="Ettus",thistype="EttusCmd",id1="NotSerial",id2="EttusOpenCloseFile")
        print("Open ID is " + str(fileOpenCloseID))
        msg=can.Message(is_extended_id=True,arbitration_id=fileOpenCloseID.IDVal(),data=openCloseData)
        print("Open Msg="+str(msg))
        self.thisBus.send(msg)
        self.WaitForAck(fileNum)
        file=open(fileName,"rb")
        readBytes = bytearray(file.read(2048))
        while(len(readBytes) != 0):
            segmentLength = len(readBytes)
            fileLength = fileLength + segmentLength
            print(fileLength)
            fileSendID = CANId(dest="Ettus",thistype="EttusCmd",id1="EttusFileData")
            print("Sending Long Message")
            self.SendLongMessage(readBytes,fileSendID)
            #If the total if the segment length that we just sent goes into the maximum chunk
            #of a long file message, then we wait for an ack.  If it is shorter than the maximum
            #chunk, then the receiving side won't know and we have to wait for an ack based on the
            #close.
            print("Return from SendLongMessage")
            if(segmentLength > 2040): #We have sent the maximum long message
                self.WaitForAck(fileNum)
            readBytes = bytearray(file.read(2048))
            print("More file read, length = {:d}".format(len(readBytes)))
        file.close()
        print("Close with leng={:d}; data={:d},{:d},{:x},{:x},{:x}".format(fileLength,1,fileNum,fileLength&255,
                                                           (fileLength>>8)&255,
                                                           (fileLength>>16)&255))
        openCloseData = bytearray([1,fileNum,fileLength&255,(fileLength>>8)&255,(fileLength>>16)&255])
        msg=can.Message(is_extended_id=True,arbitration_id=fileOpenCloseID.IDVal(),data=openCloseData)
        self.thisBus.send(msg)
        self.WaitForAck(fileNum)
        
    def SendLongMessage(self,longData,canId):
        #longData is a bytearray
        #canId is a CANId
        CANSernum = 0
        dataLen = len(longData)
        maxMessage = int((dataLen-1)/8)+1
        firstByte = 0
        lastByte = 7
        if(dataLen > 2048):
            raise Exception("Long message can not be longer than 2048")
        for msgnum in range(maxMessage):
            firstByte = msgnum*8
            lastByte = firstByte+8
            print("Sending chunk number {:d}".format(msgnum))
            canId.SetID2Numeric(msgnum) #Set the message number for this long message into the ID
            msg=can.Message(is_extended_id=True,arbitration_id=canId.IDVal(),
                                data=longData[firstByte:lastByte])
            print(msg)                
            self.thisBus.send(msg)

################################################
#  Here is where the program starts running
################################################
fileText = "sendCANFile.py" #input("Enter File to send:")
print (fileText)
filename = os.path.join(os.path.expanduser("~/PythonPrograms"),fileText)
id=CANId(dest="LIHU",priority=0)
idMask=CANId(dest="AllBits",priority=0)
canFilters=[{"can_id":id.IDVal(),"can_mask":idMask.IDVal(),"extended":True}]
print("ID="+hex(id.IDVal()))
print("Mask="+hex(idMask.IDVal()))
#thisBus = GolfCanBus(filters=canFilters)
thisBus = GolfCanBus(filters=None)

thisBus.SendFile(3,filename)
