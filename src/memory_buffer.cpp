/*
    \file   memory_buffer.c

    \brief  KSZ8851SNL ethernet controller buffer source file.

    (c) 2018 Microchip Technology Inc. and its subsidiaries.

    Subject to your compliance with these terms, you may use Microchip software and any
    derivatives exclusively with Microchip products. It is your responsibility to comply with third party
    license terms applicable to your use of third party software (including open source software) that
    may accompany Microchip software.

   THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER  
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY
    IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS
    FOR A PARTICULAR PURPOSE.

   IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP
    HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO
    THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL
    CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT
    OF FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS
    SOFTWARE.
*/

// #include "../mcc_generated_files/mcc.h"
#include "ksz8851snl.h"
#include "memory_buffer.h"
#include <string.h>
#include "tcplib/tcpip_types.h"
#include "ethernet_driver.h"
#include "tcplib/network.h"

static uint8_t TXBuffer[TXBUFFER_SIZE] __attribute__((aligned(2)));
static uint8_t RXBuffer[RXBUFFER_SIZE] __attribute__((aligned(2)));
ksz_buffers dataBuffers;

/**
 * Initialize the TX memory buffer
 */
void TXBuffer_Init(void)
{
    memset(TXBuffer, 0, TXBUFFER_SIZE);
    dataBuffers.txBuffer.start = TXBuffer;
    dataBuffers.txBuffer.bufferLength = TXBUFFER_SIZE;
    dataBuffers.txBuffer.readPointer = dataBuffers.txBuffer.start;
    dataBuffers.txBuffer.dataLength = 0;   
}

/**
 * Initialize the RX memory buffer
 */
void RXBuffer_Init(void)
{
    memset(RXBuffer, 0, RXBUFFER_SIZE);
    dataBuffers.rxBuffer.start = RXBuffer;
    dataBuffers.rxBuffer.bufferLength = RXBUFFER_SIZE;
    dataBuffers.rxBuffer.readPointer = dataBuffers.rxBuffer.start;
    dataBuffers.rxBuffer.dataLength = 0;     
}
/**
 * Receive data from the hardware buffer
 * @param length
 */
uint16_t BufferReceive(memoryBuffer *buffer, uint16_t length)
{
    PRINTF("<%s len: %u>", __FUNCTION__, length);
    uint8_t *writePointer = buffer->readPointer + buffer->dataLength;
    ETH_SPI_READ_BLOCK(writePointer, length);
    buffer->dataLength += length;
    writePointer += length;
    //Aligning the data frame to 32 bit boundary
    for(; length%4;length++)
    {
        *writePointer++ = ETH_SPI_READ8();
        buffer->dataLength++;
    }
    return length;
}

/**
 * Read 1 byte from memory buffer
 * @return
 */
uint8_t BufferRead8(memoryBuffer *buffer)
{
    buffer->dataLength--;
    uint8_t ret = *buffer->readPointer++;
    // PRINTF("<%s : %u, dLen: %u, rdPtr: %08X>", __FUNCTION__, ret, buffer->dataLength, buffer->readPointer);
    PRINTF("<%s : %u>", __FUNCTION__, ret);
    return ret;
}

static uint8_t data_[4];
/**
 * Read 2 byte from memory buffer
 * @return
 */
uint16_t BufferRead16(memoryBuffer *buffer)
{
    buffer->dataLength -= 2;
    data_[0] = *buffer->readPointer++;
    data_[1] = *buffer->readPointer;
    uint16_t ret = ((data_[0] << 8) | (data_[1]));
    PRINTF("<%s : %04X>", __FUNCTION__, ret);
    return ret;
}

/**
 * Read 4 byte from memory buffer
 * @return
 */
uint32_t BufferRead32(memoryBuffer *buffer)
{
    buffer->dataLength -= 4;
    for(uint16_t i=0; i<4; i++)
        data_[i] = *buffer->readPointer++;

    uint32_t ret = ((uint32_t)data_[0] << 24) | ((uint32_t)data_[1] << 16) | (data_[2] << 8) | (data_[3]);
    PRINTF("<%s : %08X>", __FUNCTION__, ret);
    return ret;
}

/**
 * Read block of data from memory buffer
 * @return
 */
uint16_t BufferReadBlock(memoryBuffer *buffer, char *data, uint16_t length)
{
    if (length > buffer->dataLength)
        length = buffer->dataLength;
    buffer->dataLength -= length;

    for(uint16_t i=0; i<length; i++)
        *(uint8_t*)data++ = *buffer->readPointer++;

    PRINTF("<%s> : %u", __FUNCTION__, length);
    return length; 
}

/**
 * Write 1 byte to memory buffer
 * @param data
 */
void BufferWrite8(memoryBuffer *buffer, uint8_t data)
{
    *(buffer->readPointer + buffer->dataLength++) = data;
    PRINTF("<%s> : %02X, dLen: %u", __FUNCTION__, data, buffer->dataLength);
}

/**
 * Write 2 byte to memory buffer
 * @param data
 */
void BufferWrite16(memoryBuffer *buffer, uint16_t data)
{
    uint8_t *dataEnd = buffer->readPointer + buffer->dataLength;
    *dataEnd++ = data >> 8;
    *dataEnd = data;
    buffer->dataLength += 2;
    PRINTF("<%s> : %04X, dLen: %u", __FUNCTION__, data, buffer->dataLength);
}

/**
 * Write 4 byte to memory buffer
 * @param data
 */
void BufferWrite32(memoryBuffer *buffer, uint32_t data)
{
    uint8_t *dataEnd = buffer->readPointer + buffer->dataLength;
    *dataEnd++ = data >> 24;
    *dataEnd++ = data >> 16;
    *dataEnd++ = data >> 8;
    *dataEnd++ = data;
    buffer->dataLength += 4;
    PRINTF("<%s> : %08X, dLen: %u", __FUNCTION__, data, buffer->dataLength);
}

/**
* Write block of data to memory buffer
* @param data
* @param length
*/
uint16_t BufferWriteBlock(memoryBuffer *buffer, char* data, uint16_t length)
{
    PRINTF("<%s> len: %u", __FUNCTION__, length);
    uint8_t *bufferEnd = buffer->start + buffer->bufferLength;
    uint8_t *dataEnd = buffer->readPointer + buffer->dataLength;
    uint16_t free = bufferEnd - dataEnd;
    if (length > free)
        length = free;
    buffer->dataLength += length;

    for(uint16_t i=0; i<length; i++)
        *dataEnd++ = *data++;

    return length;
}

uint16_t BufferWriteBlockX(memoryBuffer *buffer, char data, uint16_t length)
{
    uint8_t *bufferEnd = buffer->start + buffer->bufferLength;
    uint8_t *dataEnd = buffer->readPointer + buffer->dataLength;
    uint16_t free = bufferEnd - dataEnd;
    if (length > free)
        length = free;
    buffer->dataLength += length;

    for(uint16_t i=0; i<length; i++)
        *dataEnd++ = data;

    PRINTF("<%s> data: %02X, len: %u", __FUNCTION__, data, length);
    return length;
}

/**
 * Write a string to memory buffer
 * @param data
 */
 uint16_t BufferWriteString(memoryBuffer *buffer, const char *data)
{
    return BufferWriteBlock(buffer, (char*)data, strlen(data));
}

/**
* Replace the data at the specified offset
* @param data
* @param length
* @param offset
*/
void BufferReplaceAtOffset(memoryBuffer *buffer, char *data, uint16_t length, uint16_t offset)
{
    PRINTF("<%s> len: %u, off: %u", __FUNCTION__, length, offset);
    offset = (offset + 4); // sizeof(controlWord) + sizeof(packetLength)
    uint8_t *dataEnd = (buffer->start + offset);

    for(uint16_t i=0; ((i<length) && (i<buffer->dataLength)); i++)
        *dataEnd++ = *data++;
}

/**
* Copy data from RX memory buffer to TX memory buffer
* @param length
* @return
*/
error_msg BufferRXtoTXCopy(memoryBuffer *rxbuffer, memoryBuffer *txbuffer, uint16_t length)
{
    PRINTF("<%s> %u", __FUNCTION__, length);
    txbuffer->dataLength += length;
    uint8_t *txDataEnd = txbuffer->readPointer + txbuffer->dataLength;
    uint8_t *ptr = rxbuffer ->readPointer;

    while (length--) 
        *txDataEnd++ = *ptr++;

    return SUCCESS;     
}

/**
 * Get 1 byte from memory buffer. The read pointer will not change.
 * @param offset
 * @return
 */
uint8_t BufferPeekByte(memoryBuffer *buffer, uint16_t offset)
{
    return *(buffer->readPointer + offset);
}

/**
 * Dump the specified length of data
 * @param length
 */
void BufferDump(memoryBuffer *buffer, uint16_t length)
{
    if (length > buffer->dataLength)
        length = buffer->dataLength;
    PRINTF("<%s> len:  %u", __FUNCTION__, length);
    buffer->dataLength -= length;
    buffer->readPointer += length;
}

/**
* Send data from memory buffer to hardware buffer for transmitting
*/
void BufferSend(memoryBuffer *buffer)
{
    uint16_t packetLength = buffer->dataLength;
    buffer->dataLength = 0;
    PRINTF("<%s> %u", __FUNCTION__, packetLength);
    // Update final packet length to memory buffer before transmitting
    *(uint16_t*)(buffer->start+2) = packetLength;
    // send buffered data
    ETH_SPI_WRITE_BLOCK(buffer->readPointer, packetLength);
    //Aligning the data frame to 32 bit boundary
    for(; (packetLength%4) ; packetLength++)
    {
        ETH_SPI_WRITE8(0);
    }
    Serial.println("sent bytes:");
    uint8_t * b = buffer->start;
    for (uint16_t i = 0; i<packetLength; i+=10) {
        // PRINTF("%02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\n",
        //     b[i],b[i+1],b[i+2],b[i+3],b[i+4],b[i+5],b[i+6],b[i+7],b[i+8],b[i+9]);
        for (uint8_t j = 0; j<10; j++) {
            uint8_t val = b[i+j];
            if (val<16) Serial.print("0");
            Serial.print(val, HEX);
            Serial.print(" ");
        }
        Serial.println();
    }
    // for (uint16_t i = 0; i<packetLength; i+=10) {
    // }

}
