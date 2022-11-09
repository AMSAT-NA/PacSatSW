/*
 * TelemetryBufferMgmt.c
 *
 *  Created on: Feb 5, 2014 (moved from downlinkControl)
 *      Author: Burns Fisher, W2BFJ
 *  Rewritten for Golf RT-IHU Feb 14, 2020, WB1FJ
 */

/***************************************************************************************
 *    Telemetry buffer management routines
 *
 * These routines manage the buffers for low speed telemetry, which are statics:
 * DownlinkBuffers[2][size].  Each buffer will contain a single downlink frame plus
 * the FEC parity, all encoded with the 8b10b algorithm.
 *
 * Getting a telemetry buffer either for read or write must always be done through
 * these routines since this routine also manages the double buffer reading. (The
 * downlinkControl task, manages the double buffer writing)
 *
 * See TelemetryBufferMgmt.h for individual descriptions
 *
 *
 ***************************************************************************************/


#include <pacsat.h> /* Must precede stdio */
#include "stdio.h"
#include "telemetryCollectionInterface.h"
#include "interTaskNotify.h"
#include "downlink.h"
#include "MET.h"
#include "watchdogSupport.h"
#include "nonvolManagement.h"
#include "TelemEncoding.h"
#include "TelemetryBufferMgmt.h"
#include "DownlinkControl.h"


#undef assert
#define assert(boolean,number) {if(!(boolean))ReportError(UnexpectedBehavior,TRUE,PortNumber,number);}


/*
 * These variables are "local" to the buffer management routines.  The buffer
 * variable values are 0 (none), 1 (buffer 1), 2 (buffer 2), 3 (buffer 1 and 2).
 * Remember that the buffers are *called* buffer 1 and 2.  Only when you index an
 * array do you use '0' for buffer 1 and '1' for buffer 2.  That's so we can OR
 * and AND the buffers in cases where we want to talk about both or none of them.
 */


/* These are the actual telemetry buffers */
static uint32_t DownlinkBuffers[2][FEC_BUFFER_LONGWORDS]; /* Largest Possible Size */
static uint16_t BufferSize[2]; /* How much data is in each of the two buffers */
static bool BufferIsLast[2];
/* These are bitmaps that keep track of buffer states */
static uint8_t  ReadyBuffers=0,StaleBuffers=3,BufferBeingEmptied=0,BufferBeingFilled=0;

#define BUFFER0 1
#define BUFFER1 2
#define BOTH_BUFFERS 3
/*
 * #define NO_BUFFERS 0 -- This is defined in TelemetryBufferMgmt.h since
 * it is used by callers as well.
 */

/* And this keeps track of SingleFrame and Stopped states */


static bool TelemetryStopped=true,LastFrameInProgress=false;


/* Forward Routine Definitions */
static void SendTelemRequest(void);

void StopTelemetryProcessing(void){
    /*
     * No more calls to get telemetry, and when audio asks for more, it will get
     * nothing back.  If a buffer is being emptied we can't do anything about that,
     * but when it asks for more, we'll get it back.
     */

    ReadyBuffers = NO_BUFFERS;
    BufferBeingFilled = NO_BUFFERS;
    StaleBuffers = (~BufferBeingEmptied) & BOTH_BUFFERS;
    TelemetryStopped = true;
}

void InitRestartTelemetryQueues(void){
    /* Let telemetry be sent, but don't change frame mode */
    ReadyBuffers = NO_BUFFERS;
    BufferBeingFilled = NO_BUFFERS;
    StaleBuffers = (~BufferBeingEmptied) & BOTH_BUFFERS;
    TelemetryStopped = false;
    LastFrameInProgress = false;
}

FECBufferReturn GetFECBufferToDownlink(void){
    FECBufferReturn bufferToReturn;
    /*
     * This routine is called by Audio when it has run out of data to modulate and downlink.
     */
    assert(BufferBeingEmptied < BOTH_BUFFERS,1) /* Only one buffer at a time should be getting emptied*/
    StaleBuffers |= BufferBeingEmptied; /* The one Audio *was* emptying is now stale */

    /*
     * Check if we are supposed to be done!
     */
    if(LastFrameInProgress){
        bufferToReturn.buffer = 0;
        bufferToReturn.length = 0;
        bufferToReturn.lastFrame = true; //Just to make sure they understand why there is nothing here!
        LastFrameInProgress = false;
        TelemetryStopped = true;
    } else {

        /*
         * Now find a buffer to return.
         */
        assert(ReadyBuffers <= BOTH_BUFFERS,2) /* Anything greater than both is bogus */
	        assert(ReadyBuffers != BOTH_BUFFERS,3)

	        if(ReadyBuffers == NO_BUFFERS){
	            /*
	             * Nothing is ready to send.  Sorry!
	             */
	            if(BufferBeingEmptied != NO_BUFFERS){
	                BufferBeingEmptied=NO_BUFFERS;
	            }
	            BufferBeingEmptied=NO_BUFFERS;
	            bufferToReturn.buffer = (void *)NO_BUFFERS;
	            bufferToReturn.length = 0;
	            bufferToReturn.lastFrame = false;

	        } else if(ReadyBuffers & BUFFER0){
	            /*
	             * Buffer 0 is ready
	             */
	            BufferBeingEmptied = BUFFER0;     /* If buffer 0 is available, use that... */
	            ReadyBuffers &= (~BUFFER0);       /* ... and clear that bit out of ready  */
	            bufferToReturn.buffer = DownlinkBuffers[0];
	            bufferToReturn.length = BufferSize[0];
	            bufferToReturn.lastFrame = BufferIsLast[0];
	            LastFrameInProgress = BufferIsLast[0];
	        } else {
	            /*
	             * Buffer 1 is ready
	             */
	            BufferBeingEmptied = BUFFER1;
	            ReadyBuffers &= ~(BUFFER1);
	            bufferToReturn.buffer = DownlinkBuffers[1];
	            bufferToReturn.length = BufferSize[1];
	            bufferToReturn.lastFrame = BufferIsLast[1];
	            LastFrameInProgress = BufferIsLast[1];
	        }

        /*
         * End of buffer management.
         * If we are still collecting telemetry and if there are no more
         * ready buffers or buffers being filled, ask for one to be filled.
         */
        if(LastFrameInProgress || TelemetryStopped || (StaleBuffers == NO_BUFFERS))
        {
            // Do nothing if any of the above conditions are true
        } else {
            // Otherwise, ask for more telemetry data to fill another buffer
            SendTelemRequest();
        }
    }
    return bufferToReturn;
}

uint32_t *GetFECBufferToFill(void){
    /*
     * When downlinkControl gets a message to generate some telemetry data,
     * this is what it calls to manage the buffers.
     */
    uint32_t *bufferToReturn;

    assert(ReadyBuffers <= BOTH_BUFFERS,4)
    assert(StaleBuffers <= BOTH_BUFFERS,5)
    assert(BufferBeingEmptied < BOTH_BUFFERS,6)

    if((ReadyBuffers == BOTH_BUFFERS) || (BufferBeingFilled != NO_BUFFERS)) {
        /*
         * The buffers are all full or already being filled.  Nothing
         * to do.
         */
        bufferToReturn = (void *)NO_BUFFERS;
    } else if((StaleBuffers & BUFFER0) != 0){
        /*
         * Buffer 0 is available to use.  Take it out of the stale list,
         * put it into the filling list.
         */
        StaleBuffers &= (~BUFFER0);
        BufferBeingFilled = BUFFER0;
        bufferToReturn = DownlinkBuffers[0];

    } else if (StaleBuffers & BUFFER1){
        /*
         * Buffer 1 is available to use.
         */
        StaleBuffers &= (~BUFFER1);
        BufferBeingFilled = BUFFER1;
        bufferToReturn = DownlinkBuffers[1];
    }
    else bufferToReturn = NO_BUFFERS; // It must be that one buffer is full, the other being emptied
    return bufferToReturn;
}
void ReportFrameReadyToSend(uint16_t length,bool lastFrame){
    /*
     * Callback when we have filled a frame with data
     * after getting it from GetFECBufferToFill.
     */
    assert(BufferBeingFilled < BOTH_BUFFERS,7)
	        assert(BufferBeingFilled != NO_BUFFERS,8)

	        ReadyBuffers |= BufferBeingFilled; /* Otherwise, we'll keep it! */

    if(BufferBeingFilled & BUFFER0){
        BufferSize[0] = length;
        BufferIsLast[0] = lastFrame;
    } else {
        BufferSize[1] = length;
        BufferIsLast[1] = lastFrame;
    }
    BufferBeingFilled = NO_BUFFERS;
}

static void SendTelemRequest(void){
    /* We are out of full buffers.  Ask downlinkControl for more */
    Intertask_Message message;
    bool status;
    message.MsgType = DwnlnkCollectTelemetry;
    status = NotifyInterTask(ToDownlinkControl, 0, &message);
    if(!status)
        ReportError(ControlQueueOverflow,true,CharString,(int)"TelemTimer");
}
void PrintBufferData(){
    printf("Buffer State: Stopped=%d,LastInProgress=%d,Ready=%d,Stale=%d,Filling=%d,Emptying=%d\n\r",
           TelemetryStopped,LastFrameInProgress,ReadyBuffers,StaleBuffers,BufferBeingFilled,
           BufferBeingEmptied);

}
