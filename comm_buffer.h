/*
 * comm_buffer.h
 *
 *  Created on: 17-11-2013
 *      Author: Korzo
 */

#ifndef COMM_BUFFER_H_
#define COMM_BUFFER_H_

//-----------------------------------------------------------------

#define COMM_BUFFER_SIZE        32

#define MSG_POS_START           0
#define MSG_POS_LENGTH          1
#define MSG_POS_CMD             2
#define MSG_POS_RESPONSE        3
#define MSG_POS_DATA            4

#define MSG_START               0x99

//-----------------------------------------------------------------

#define COMM_BUFFER_READ_T(buf,dst,type)   (commBufferRead((buf), (unsigned char*)(dst), sizeof(type)))
#define COMM_BUFFER_WRITE_T(buf,src,type)  (commBufferWrite((buf), (unsigned char*)(src), sizeof(type)))

//-----------------------------------------------------------------

typedef struct
{
    unsigned char data[COMM_BUFFER_SIZE];
    unsigned char *writePtr;
    unsigned char *readPtr;
    unsigned char command;
    unsigned char response;
    unsigned int length;

} CommBuffer;

//-----------------------------------------------------------------

void commBufferReset(CommBuffer *buf);

void commBufferWrite(CommBuffer *buf, unsigned char *src, unsigned int len);
void commBufferResetWrite(CommBuffer *buf);

void commBufferRead(CommBuffer *buf, unsigned char *dst, unsigned int len);
void commBufferResetRead(CommBuffer *buf);

void commBufferWriteHeader(CommBuffer *buf);
void commBufferReadHeader(CommBuffer *buf);
void commBufferCreateHeader(CommBuffer *buf, unsigned char cmd, unsigned char resp);

//-----------------------------------------------------------------

#endif /* COMM_BUFFER_H_ */
