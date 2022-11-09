/*
 * Telemetry Audio Module
 * by Burns Fisher, AMSAT-NA, 2012
 */
/*
 * Major change for Golf-T, RT-IHU March, 2020
 * Burns Fisher, WB1FJ, AMSAT-NA
 *
 * Note that telemetry is driven from this module after it gets started.  An interrupt
 * from the AX5043 sends a message to this task, which starts pulling queued telemetry
 * from the queue module.  The telemetry queues are double buffers, so when one is taken
 * for use by this task, a the telemetry module sends a request to DownlinkControl for another
 * packet to be readied.
 *
 */



/*
 * The Fox/LIHU/LTM-1 version works similarly, except that this module is responsible for
 * sending samples to the PSK modulator rather than bytes to the AX5043 queue.  And the LIHU
 * version is still hack on hack as we switched from FM DUV to SSB/PSK.  Hopefully this is a
 * bit cleaner!
 */

/* Includes ------------------------------------------------------------------*/

#include <pacsat.h>
#include "stdint.h"
#include "stdio.h"

#include "FreeRTOS.h"
#include "os_task.h"
#include "os_semphr.h"

#include "gpioDriver.h"
#include "TelemetryRadio.h"
#include "watchdogSupport.h"
#include "interTaskNotify.h"
#include "DownlinkControl.h"
#include "nonvolManagement.h"

#include "MRAMmap.h"
#include "TelemetryBufferMgmt.h"
#include "CommandTask.h"
#include "downlink.h"
#include "ax5043_access.h"

#include "ax5043.h"
#include "ax5043-2M-AFSK-externs.h"


//#define TIMER_REPLACES_5043
// DMA_BUFFER_LEN is the size of ONE buffer.  The array is both
// of the double buffers.

#define DMA_BUFFER_LEN 48
#define minFifoFree 20
#define MAX_TELEM_BYTES 2000
#define TONE_BUFFER_BYTES 256

FECBufferReturn newBuffer;

static enum {
             /* [
              * On the LIHU, these decide various types of telemetry.  Here, we are only using none, tone, and
              * TelemLowSpeed.  You could use Init if you needed to and then switch it to TelemLowSpeed after
              * the whatever the init is supposed to do is done.  In fact, you could use init to send an initial
              * pseudo-random number sync and then always send one after each frame.
              */
             TelemNone=0,
             TelemTone,
             TelemInit,
             Telem1200BPSK,
             TelemSquare
} TelemetryType;
static bool InitTelemetry = true;

/* Stuff for the uplink software commands*/

#define BINARY(value) ((value > UPLINK_MARK_VALUE)? UPLINK_COMMAND_MARK : UPLINK_COMMAND_SPACE)

// Forward routines

static void SendTelemComplete(void);
static void FakeDCTInterrupt(void);


static uint8_t PAPowerFlagCnt=0,DCTPowerFlagCnt=0;

static int sendingPacket = 0;
bool startWithSync;
bool endWithSync;

int currentFrameWords;
int currentFrameBytes;
int currentFrameRemainder;
int telemWordNum = 0;
static int telemByteNum = 0;
static int telemBytes;
static int partial;
uint16_t current10bWord;
uint32_t *fecFrameToSend;
uint16_t irqs;
int bytesRead = 0;

uint8_t byteBuf[MAX_TELEM_BYTES];

void copyAndPack(uint8_t *byteBuf, FECBufferReturn *newBuffer, bool *startWithSync, bool *endWithSync, int *bytes, int *partial);



portTASK_FUNCTION_PROTO(TelemetryRadioTask, pvParameters)  {
    int j;
    bool lastPacketQueued=false;

    vTaskSetApplicationTaskTag((xTaskHandle) 0, (pdTASK_HOOK_CODE)RadioWD );
    InitInterTask(ToRadio, 10);


    ResetAllWatchdogs();

    while (1) { /* Main audio processing loop.  Yes, it goes forever */
        Intertask_Message messageReceived;
        int status;
        ReportToWatchdog(CurrentTaskWD);
        status = WaitInterTask(ToRadio, SECONDS(2), &messageReceived);
        ReportToWatchdog(CurrentTaskWD);
        if(status == 1){
            //int waiting=WaitingInterTask(ToRadio);
            //if(waiting != 0){
            //    debug_print("MessagesWaiting=%d\n",WaitingInterTask(ToRadio));
            //}
            switch(messageReceived.MsgType){
            case DCTPowerFlagMsg:
                debug_print("AX5043 Power Interrupted\n");
                DCTPowerFlagCnt++;
                break;
            case PAPowerFlagMsg:
                debug_print("Power Amp Power Interrupted\n");
                PAPowerFlagCnt++;
                break;

            case DCTInterruptMsg: {

                if ((ax5043ReadReg(AX5043_PWRMODE) & 0x0F) == AX5043_PWRSTATE_FULL_RX) {
                    //printf("Interrupt while in FULL_RX mode\n");
                    //printf("IRQREQUEST1: %02x\n", ax5043ReadReg(AX5043_IRQREQUEST1));
                    //printf("IRQREQUEST0: %02x\n", ax5043ReadReg(AX5043_IRQREQUEST0));
                    //printf("FIFOSTAT: %02x\n", ax5043ReadReg(AX5043_FIFOSTAT));
                    if ((ax5043ReadReg(AX5043_FIFOSTAT) & 0x01) == 0) {
                        //printf("FIFO not empty\n");
                        //ax5043WriteReg(AX5043_FIFOSTAT, 3); // clear FIFO data & flags - Temporary until we can process the FIFO
                        //printf("After emptying FIFOSTAT: %02x\n", ax5043ReadReg(AX5043_FIFOSTAT));
                        bytesRead = receive_packet_2m();
                        //printf("bytes read from FIFO: %d\n", bytesRead);

                        //Clear status bits
                        //ax5043ReadReg(AX5043_RADIOEVENTREQ1);
                        //ax5043ReadReg(AX5043_RADIOEVENTREQ0);


                        //printf("bytes received: %02x %02x %02x\n", axradio_rxbuffer_2m[0], axradio_rxbuffer_2m[1], axradio_rxbuffer_2m[2]);
                        /* todo
                         * If we are in receive mode, this might be a legit command
                         * so extend the timer.  (We might want to do this differently and
                         * wait till we were more sure of something real).  In any case, start
                         * receive timer with a true argument only has an effect if the timer is
                         * actually running
                         */
                        StartReceiveTimer(true);
                        incomingRawSoftwareCommand(axradio_rxbuffer_2m);


                    }
                    break;
                } else {
                    //printf("AX5043 Interrupt in pwrmode: %02x\n", ax5043ReadReg(AX5043_PWRMODE));
                }

                /*
                 * For now assume an interrupt means the buffer is getting low.  In the future, we may enable other ax5043 interrupts.
                 */

                int numbytes = fifo_free();
                int fifoFree = numbytes;
                int flags = 0x18; // NOCRC and RAW
                //debug_print("5043 Interrupt\n");
                if(TelemetryType == MixerSilence){
                    //debug_print("Telemetry silence--return\n");
                    break;
                }
                if(InitTelemetry){
                    //debug_print("Initializing radio variables\n");
                    InitTelemetry = false;
                    lastPacketQueued = false;
                    sendingPacket = false;
                    startWithSync = false;
                    endWithSync = false;
                    sendingPacket = 0;
                    telemByteNum = 0;
                    // Now continue on in whatever telemetry type was specified
                }

                // if the previous packet was the final packet and the FIFO is now empty,
                // i.e. there are 256 bytes in the FIFO, turn off interrupts and tell
                // downlink control we are done (d/c will turn off power)

                if (lastPacketQueued && (numbytes == 256)) {
                    //debug_print("Last packet queued\n");

                    // The next two calls will shut the telemetry and the DCT

                    AudioSetMixerSource(MixerSilence);
                    lastPacketQueued = 0;

                    // Disable ax5043 interrupts
                    irqs = 0;
                    ax5043WriteReg(AX5043_IRQMASK0, irqs & 0xff);
                    ax5043WriteReg(AX5043_IRQMASK1, irqs >> 8);
                    /*
                     * Make sure it has plenty of time to complete whatever it is sending now
                     */
                    vTaskDelay(CENTISECONDS(15));
                    SendTelemComplete();  // Tell Downlink we are done.  It will power off the radio
                    break;
                }

                if (!sendingPacket) {
                    //debug_print("Not sending packet, fifo size is %d\n",numbytes);
                    if(TelemetryType == TelemTone){
                        for(j=0;j< TONE_BUFFER_BYTES;j++){
                            byteBuf[j]= 0xFF; //All 1 bits will end up sending the same phase (as would all 1 bits)
                        }
                        telemBytes = TONE_BUFFER_BYTES;
                        lastPacketQueued = false;
                        startWithSync = false;
                        endWithSync = false;
                        sendingPacket = 1;
                        telemByteNum = 0;
                        //debug_print("Filled tone buffer with %d bytes\n",
                        //            TONE_BUFFER_BYTES);
                    } else if(TelemetryType == TelemSquare){
                         for(j=0;j< TONE_BUFFER_BYTES;j++){
                            byteBuf[j]= 0x00; //101010 bits sent
                        }
                        telemBytes = TONE_BUFFER_BYTES;
                        lastPacketQueued = false;
                        startWithSync = false;
                        endWithSync = false;
                        sendingPacket = 1;
                        telemByteNum = 0;
                    } else {

                    newBuffer = GetFECBufferToDownlink();
                    fecFrameToSend = newBuffer.buffer;
                    
                    if (fecFrameToSend == (void *) NO_BUFFERS) {
                        //debug_print("No Buffer\n");
                        // This generally happens once for the last frame of safe mode
                        break;
                    } else {
                        //debug_print("Got new buffer, length=%d, lastFrame=%d\n", newBuffer.length,newBuffer.lastFrame);
                        lastPacketQueued = newBuffer.lastFrame;
                        copyAndPack(byteBuf, &newBuffer, &startWithSync, &endWithSync, &telemBytes, &partial);


                        if (startWithSync) {
                            //printf("Enqueueing starting SYNC\n");
                            fifo_send_sync(0);
                        }

                        sendingPacket = 1;
                        telemByteNum = 0;

                    }
                    }
                } 
                    
                // Already sending a packet, so continue queuing bytes if there's room

                //debug_print("Sending packet\n");

                //printf("\nnumbytes: %d\n", numbytes);
                //printf("minFifoFree: %d\n", minFifoFree);
                //printf("telemBytes: %d\n", telemBytes);
                //printf("telemByteNum: %d\n", telemByteNum);

                if (numbytes >= minFifoFree) {

                    // how many bytes are we going to enqueue?

                    numbytes -= 10; // Leave some room for the header and sync

                    if (numbytes >= (telemBytes - telemByteNum)) { // last chunk?
                        numbytes = telemBytes - telemByteNum;

                        // last chunk of buffer

                        //printf("Last chunk,last queued=%d\n",lastPacketQueued);
                        //printf("Setting sendingPacket to 0\n");

                        sendingPacket = 0;

                        // Handle partial last byte

                        if (partial) {

                            //printf("Partial\n");
                            // add stop bit <partial> bits from the left
                            // and set the flag for partial last byte

                            byteBuf[telemBytes - 1] |= (1 << (7 - partial));
                            flags |= 0x04;
                        }

                        if (lastPacketQueued) {
                            /*
                             * Disable ax5043 interrupts
                             * Disable FIFO low interrupt
                             * Enable FIFO empty interrupt
                             * Re-enable interrupts
                             */
                            ax5043WriteReg(AX5043_PINFUNCIRQ, 0x0); //disable IRQs

                            irqs = 1; // FIFO not Empty (inverted below)
                            ax5043WriteReg(AX5043_IRQMASK0, irqs & 0xff);
                            ax5043WriteReg(AX5043_IRQMASK1, irqs >> 8);
                            ax5043WriteReg(AX5043_IRQINVERSION0, 0x1); //invert FIFO not empty interrupt

                            ax5043WriteReg(AX5043_PINFUNCIRQ, 0x3); //enable IRQs
                        } else if ((fifoFree + numbytes < 150) || numbytes < 10){ //150 should be a threshold constant
                            /*
                             * We are not going above the threshold in the fifo, so we will not get
                             * an interrupt when we go below it.  So we fake an interrupt to ourselves
                             */
                            //debug_print("Faking an interrupt--\n");
                            FakeDCTInterrupt();
                        }

                        // What happens if we don't enqueue enough to clear the IRQ?
                        // The answer is that we send ourselves a message to look again (see above)

                    }

                    //printf("Queuing Buffer with %d bytes\n", numbytes);

                    fifo_queue_buffer(byteBuf+telemByteNum, numbytes, flags);

                    telemByteNum += numbytes;
                            
                    // Check if trailing sync needed

                    if ((telemBytes == telemByteNum) && endWithSync) {
                        //printf("Sending final sync\n");
                        fifo_send_sync(0);
                    }
                                
                    //printf("Committing\n");
                    fifo_commit();
              
                } // Else there was not enough room in the FIFO, but it will interrupt again with more room

                //break;
            } // case DCTInterrupt
            } // switch on message type
        } // if status == 1
    } // while (1)
} // portTASK_FUNCTION_PROTO 

void AudioSetMixerSource(Mixer_Source src){
    /*
     * This routine chooses which audio sources are to be sent to the
     * transmitter.  Probably not all of these are required on Golf
     */
    static Mixer_Source currentSrc = MixerSilence;
    InitTelemetry = (src != currentSrc); //If we are changing sources, let the variables get initted
    currentSrc = src;
    switch (src){
    case MixerSilence:
        TelemetryType = TelemNone;                      // Turn off telemetry
        break;
    case MixerTone:
        TelemetryType = TelemTone;
        break;
    case MixerSquare:
        TelemetryType = TelemSquare;
        break;
    case MixerData:
        TelemetryType = Telem1200BPSK;
    case MixerInit:
        InitTelemetry = true;
        break;

    default: break;
    }
}
static void SendTelemComplete(void){
    /* Tell downlinkControl we are done sending telemetry */
    Intertask_Message message;
    bool status;
    message.MsgType = DwnlnkTelemetryFrameComplete;
    status = NotifyInterTask(ToDownlinkControl, 0, &message);
    if(!status)
        ReportError(ControlQueueOverflow,true,CharString,(int)"TelemTimer");
}

/*
 * The following is used when we are in a position where we need to run the interrupt routine again
 * but the DCT is not going to generate an interrupt (e.g. if the bytes we have to put in do to
 * exceed the interrupt threshold
 */
void FakeDCTInterrupt(){
    Intertask_Message messageSent;
    messageSent.MsgType = DCTInterruptMsg;
    NotifyInterTask(ToRadio,0,&messageSent); //This is what a DCT interrupt will do
}

/*
 * Routines to get telemetry info
 */

uint8_t GetPAPowerFlagCnt(void){
    return PAPowerFlagCnt;
}
uint8_t GetDCTPowerFlagCnt(void){
    return DCTPowerFlagCnt;
}

void copyAndPack(uint8_t *byteBuf, FECBufferReturn *newBuffer, bool *startWithSync, bool *endWithSync, int *bytes, int *partial) {

    uint32_t *fecFrameToSend = newBuffer->buffer; 
    int start = 0;
    int i;
    uint16_t word;
    int state = 0;
    int count = 0;
  
    *startWithSync = *endWithSync = 0;

    if (newBuffer->length <= 0) return;
  
    if (Get10bFromBuffer(fecFrameToSend,0) == SYNC10b0) {
        *startWithSync = 1;
        start++;
    }

//    currentFrameWords = newBuffer.length;
//    currentFrameBytes = (currentFrameWords * 10) / 8;
//    currentFrameRemainder = (currentFrameWords * 10) % 8;

    // Check for MAX_TELEM_BYTES - TODO RBG

    //printf("In copyAndPack: newBuffer->length: %d\n", newBuffer->length);
    //printf("start: %d\n", start);
  
    for (i=start; i< (newBuffer->length); i++) {

        //printf("In loop\n");


        word = Get10bFromBuffer(fecFrameToSend,i);

#if 0
        if (i < 10) printf("%03x ", word); //DEBUG
        if (i == 10) printf("\n...\n");
        if (i > (newBuffer->length - 10)) printf("%03x ", word); //DEBUG
//#else
        printf("0x%03x ", word); //DEBUG
        if ((i % 10) == 9) printf("\n");
#endif

        if (word == SYNC10b0) {
            //printf("Found end sync inside loop, index=%d, size=%d\n",i,newBuffer->length);
            *endWithSync = 1;
            break;
        }

        switch (state) {
        case 0: {
            byteBuf[count++] = word >> 2;
            byteBuf[count] = word << 6;
            *partial = 2; // Number of high order bits in the last byte
            break;
        }
        case 1: {
            byteBuf[count++] |= word >> 4;
            byteBuf[count] = word << 4;
            *partial = 4; // Number of high order bits in the last byte
            break;
        }
        case 2: {

            byteBuf[count++] |= word >> 6;
            byteBuf[count] = word << 2;
            *partial = 6; // Number of high order bits in the last byte
            break;
        }
        case 3: {
            byteBuf[count++] |= word >> 8;
            byteBuf[count++] = word & 0xff;
            byteBuf[count] = 0;
            *partial = 0; // Last byte is full
            break;
        }
        } //switch


        if (state++ == 3) {
            state = 0;
        }

        //printf("in copyAndPack: count: %d", count);

    } // for loop

    //printf("\n"); // DEBUG
  

    // ********* if partial != 0, count is low by one ***********

    *bytes = *partial ? count+1 : count;

    //printf("At end of copyAndPack: count: %d, partial: %d, bytes: %d\n", count, *partial, *bytes);
}
 
