#include "comm_buffer.h"
#include <string.h>

//-----------------------------------------------------------------

void commBufferReset(CommBuffer *buf)
{
    buf->writePtr = &(buf->data[MSG_POS_DATA]);
    buf->readPtr = buf->writePtr;
    buf->length = 0;
}

//-----------------------------------------------------------------

void commBufferWrite(CommBuffer *buf, unsigned char *src, unsigned int len)
{
    memcpy(buf->writePtr, src, len);
    buf->writePtr += len;
    buf->length += len;
}

//-----------------------------------------------------------------

void commBufferResetWrite(CommBuffer *buf)
{
    buf->writePtr = &(buf->data[MSG_POS_DATA]);
    buf->length = 0;
}

//-----------------------------------------------------------------

void commBufferRead(CommBuffer *buf, unsigned char *dst, unsigned int len)
{
    memcpy(dst, buf->readPtr, len);
    buf->readPtr += len;
}

//-----------------------------------------------------------------

void commBufferResetRead(CommBuffer *buf)
{
    buf->writePtr = &(buf->data[MSG_POS_DATA]);
}

//-----------------------------------------------------------------

void commBufferWriteHeader(CommBuffer *buf)
{
    buf->data[MSG_POS_START]    = MSG_START;
    buf->data[MSG_POS_CMD]      = (unsigned char) buf->command;
    buf->data[MSG_POS_RESPONSE] = (unsigned char) buf->response;
    buf->data[MSG_POS_LENGTH]   = buf->length;
}

//-----------------------------------------------------------------

void commBufferReadHeader(CommBuffer *buf)
{
    buf->start     = buf->data[MSG_POS_START];
    buf->command   = (QuadCommand) buf->data[MSG_POS_CMD];
    buf->response  = (QuadCommand) buf->data[MSG_POS_RESPONSE];
    buf->length    = buf->data[MSG_POS_LENGTH];
}

//-----------------------------------------------------------------

void commBufferCreateHeader(CommBuffer *buf, QuadCommand cmd, QuadCommand resp)
{
    buf->command = cmd;
    buf->response = resp;
    commBufferWriteHeader(buf);
}
