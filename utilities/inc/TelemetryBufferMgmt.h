/*
 * TelemetryBufferMgmt.h
 *
 *  Created on: Feb 5, 2014
 *      Author: fox
 */

#ifndef TELEMETRYBUFFERMGMT_H_
#define TELEMETRYBUFFERMGMT_H_
typedef struct _BufferReturn {
	uint32_t *buffer;
	uint16_t length;
	bool lastFrame;
}FECBufferReturn;

#define NO_BUFFERS 0

/************************************************************************
 * This group of routines deal simply with getting buffers to fill or to
 * downlink, and reporting them full or empty
 ************************************************************************/

FECBufferReturn GetFECBufferToDownlink(void);
/*
 * This routine is used (generally by the audio)
 * to request a new buffer to transmit.  It can return a buffer of 0, which means there is no buffer
 * available.  (That might happen when telemetry gets marked stale.)  Calling this routine
 * also puts the buffer that was last fetched by this routine (i.e. last transmitted) on the stale list.
 * Note that if this routine returns a buffer of 0, that means the caller has to stop sending telemetry.
 * The FECBufferReturn is a structure with the buffer pointer and length.  That may go away.
 */
uint32_t *GetFECBufferToFill(void);
/*
 * This is used by downlink control to request a buffer
 * to fill with new data.  It might return 0 if all the buffers have fresh data. The structure
 * FECBufferReturn contains a buffer pointer which the caller uses to fill the data.
 */
void ReportFrameReadyToSend(uint16_t length,bool lastFrame);
/*
 * This routine says that the buffer previously returned by
 * GetFECBufferToFill is now filled and can be put on the ready list to be sent.  It must
 * be called after each call to GetFECBufferToFill (unless Get... returns 0).  If "lastFrame"
 * is set to true, then no more requests will be made fill another frame, and after this
 * frame is transmitted.  In other words, after this frame is given out via GetFECBufferToDownlink,
 * the next call to ...Downlink will return 0, and a "done" message will be sent to downlinkControl.
 */


/*****************************************************************************
 * This group of routines deals with the weird exceptions:  Starting, stopping
 * specifying exactly one or two more frames to send, etc.
 *****************************************************************************/

void StopTelemetryProcessing(void);
void InitRestartTelemetryQueues(void);
/*
 * If Stop is called, no more telemetry will be collected (and no more buffers
 * returned via GetFECBufferToDownlink until telemetry Restart is called.
 */


/*
 * These routines assume that no frame is currently being
 * sent.  It sets up so that when telemetry starts, it will send exactly one or
 * two frames with a "Frame Complete" message after
 * each one.
 */
void MarkTelemetryCompleteCurrentFrame(void);
/*
 * This allows the current frame to complete (or if
* there is nothing in progress, it will send one more).  It then sends a "Frame Complete"
* message to DownlinkControl.  It works by throwing away all data that might already
* be in the ready buffer and by specifying that there is no buffer ready (nor being filled).
* It also puts all buffers on the stale list (except for one currently being used, which
* is allowed to finish).  It sets the variable "FramesRemaining" so that control will get
* one more message when the current frame is done. To get out of stale/single frame mode
* you send a get telem message to downlink control. TODO:  Is this right?
 */

/*
 * This one is just to call from get status to get a bit of extra information for debugging
 */
void PrintBufferData();


#endif /* TELEMETRYBUFFERMGMT_H_ */
