/*
    \file   ksz8851snl.c

    \brief  KSZ8851SNL Ethernet controller driver source file.

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

// #include "mcc.h"
#include "ethernet_driver.h"
#include "ksz8851snl.h"
#include "memory_buffer.h"
#include "tcplib/network.h"
#include "tcplib/tcpip_config.h"

// Packet write in progress, not ready for transmit
#define ETH_WRITE_IN_PROGRESS           (0x0001 << 0)
// Packet complete, in queue for transmit
#define ETH_TX_QUEUED                   (0x0001 << 1)
// Flag for pool management - free or allocated
#define ETH_ALLOCATED                   (0x0001 << 2)

// adjust these parameters for the MAC...
#define RAMSIZE                         (0x47FF) //18431
#define MAX_TX_PACKET_SIZE              (1518)
#define MIN_TX_PACKET_SIZE              (64)
#define MAX_TX_PACKETS                  (20)

#define TX_STATUS_VECTOR_SIZE           (4)

// typical memory map for the MAC buffers
#define TXSTART                         (0x0000)
#define TXEND                           (0x1800) // 6144
#define RXSTART                         (0x0000) 
#define RXEND                           (0x2FFF) //12287

#define MIN_TX_PACKET                   (MIN_TX_PACKET_SIZE + TX_STATUS_VECTOR_SIZE)
#define TX_BUFFER_SIZE                  (TXEND  - TXSTART) //6144

#define TX_BUFFER_MID                   ((TXSTART + TXEND) >> 1 ) //3072

#define SetBit( bitField, bitMask )     do{ bitField = bitField | bitMask; } while(0)
#define ClearBit( bitField, bitMask )   do{ bitField = bitField & (~bitMask); } while(0)
#define CheckBit( bitField, bitMask )   (bool)(bitField & bitMask)


#define controlWord  0x0080
#define packetLength 0x0000
#define pointerMask  0x07FF

// Define a temporary register for passing data to inline assembly
// This is to work around the 110110 LSB errata and to control RDPTR WRPTR update counts

static txPacket_t  *pHead;
static txPacket_t  *pTail;
static txPacket_t txData[MAX_TX_PACKETS];
static receiveStatusVector_t rxPacketStatusVector;
// static uint16_t nextPacketPointer;
static uint8_t ethListSize;
volatile ethernetDriver_t ethData;

static error_msg ETH_SendQueued(void);
static void ETH_PacketListReset(void);
static txPacket_t* ETH_NewPacket(void);

/*******************************************************************************/

bool ETH_CheckLinkUp(void)
{
    PRINTF("<%s..>", __FUNCTION__);;
    ethData.up = ((KSZ88_RegRead16(KSZ_P1SR) & P1SR_LINK_GOOD) > 0);
    return ethData.up;
}

/**
 * "Allocate" a new packet element and link it into the chained list
 * @param 
 * @return  packet address
 */
static txPacket_t* ETH_NewPacket(void)
{
    PRINTF("<%s>", __FUNCTION__);;
    if( ethListSize == MAX_TX_PACKETS )
        return NULL;

    uint8_t index = 0;

    while( index < MAX_TX_PACKETS )
    {
        if( CheckBit(txData[index].flags, ETH_ALLOCATED) == false )
        {
            // reset all flags
            txData[index].flags = 0;                        
            // allocated = true - mark the handle as allocated
            SetBit(txData[index].flags, ETH_ALLOCATED);     

            txData[index].packetEnd = TXEND;

            txData[index].prevPacket = NULL;
            txData[index].nextPacket = pHead;

            if( pHead != NULL )
            {
                pHead->prevPacket = &txData[index];
                txData[index].packetStart = pHead->packetEnd + TX_STATUS_VECTOR_SIZE;
                    
                // Try to keep a 2byte alignment
                if( txData[index].packetStart & 0x0001 )
                {
                    // Make sure the end of the packet is odd, so the beginning of the next one is even
                    ++ txData[index].packetStart;
                }
            }
            else
            {
                txData[index].packetStart = TXSTART;
                pTail = &txData[index];
            }

            pHead = &txData[index];

            ethListSize ++;
            return (&txData[index]);
        }
        index ++;
    }
    return NULL;
}

/**
 * Reset the Ethernet Packet List
 * @param 
 * @return
 */
static void ETH_PacketListReset(void)
{
    PRINTF("<%s>", __FUNCTION__);;
    ethListSize = 0;

    pHead = NULL;
    pTail = NULL;

    uint8_t* ptr = (uint8_t*)&txData;
    uint16_t index = 0;
    while( index < (MAX_TX_PACKETS * sizeof(txPacket_t)) )
        ptr[index++] = 0;
}

/**
 * Unlink a packet element from the chained list and "deallocate" it
 * @param   packetHandle
 * @return 
 */
static void ETH_RemovePacket(txPacket_t* pPacket)
{    
    PRINTF("<%s>", __FUNCTION__);;
    if( (pPacket == NULL) || (ethListSize == 0) )
        return;

#ifdef VALIDATE_ALLOCATED_PTR
    uint8_t index = 0;
    while( index < MAX_TX_PACKETS )
    {
        if( (pPacket == &txData[index]) && (txData[index].allocated == true) )
            break;
        index ++;
    }
    if( index == MAX_TX_PACKETS )
        return;
#endif  /* VALIDATE_ALLOCATED_PTR */

    // Unlink from the chained list
    if( pPacket->nextPacket == NULL )
    {
        pTail = (txPacket_t*)pPacket->prevPacket;
        if( pTail != NULL )
            pTail->nextPacket = NULL;
    }

    if( pPacket->prevPacket == NULL )
    {
        pHead = (txPacket_t*)pPacket->nextPacket;
        if( pHead != NULL )
            pHead->prevPacket = NULL;
    }

    // Deallocate
    pPacket->flags = 0;
    pPacket->prevPacket = NULL;
    pPacket->nextPacket = NULL;

    ethListSize --;
}

volatile bool ethEventOccured;
/**
  * Callback function to be xecuted when ETH_IRQ_PIN goes low 
  */
void ETH_IRQ_Callback(void)
{
    ethEventOccured = true;
}

/**
 * Ethernet Initialization - Initializes TX/RX Buffer, MAC and PHY
 */
void ETH_Init(void)
{
    PRINTF("<%s", __FUNCTION__);;
    KSZ88_Init();

    ETH_SetMAC((uint8_t*)macAddress.array);
    attachInterrupt(ETH_IRQ_PIN, ETH_IRQ_Callback, FALLING);
    ethEventOccured = false;

   // initialize the driver variables
    ethData.error = false; // no error
    ethData.up = false; // no link
    ethData.linkChange = false;
    
    ETH_PacketListReset();

    ethData.saveRDPT = 0;
    // controlWord = 0x0080;
    // packetLength = 0;
    PRINTF("%s>", __FUNCTION__);;
}

/**
 * Poll Ethernet Controller for new events
 */
void ETH_EventHandler(void)
{
    // if (!ethEventOccured) return;
    // ethEventOccured = false;

    PRINTF("<%s", __FUNCTION__);;
    uint16_t isr = KSZ88_RegRead16(KSZ_ISR);
    printf("ETH_EventHandler ISR: %04X\n", isr);

    if ( isr & ISR_LDIS )
    {
        KSZ88_BitFieldSet(KSZ_ISR, ISR_LDIS);
        // uint16_t pmecr = KSZ88_RegRead16(KSZ_PMECR);
        // printf("Wake-up event: %X\n", (pmecr>>2)&0x0F);
        KSZ88_BitFieldSet(KSZ_PMECR, PMECR_WAKEUP_LINK);
    }

    if ( isr & ISR_LCIS )
    {
        KSZ88_BitFieldSet(KSZ_ISR, ISR_LCIS);
        ethData.linkChange = true;
        ETH_CheckLinkUp();
    }

    if ( isr & ISR_TXIS )
    {
        KSZ88_BitFieldSet(KSZ_ISR, ISR_TXIS);
        if( ethListSize > 0 )
        {
            //Send the next queued packet
            ETH_SendQueued();
        }
    }

    if ( isr & ISR_RXIS )
    {
        KSZ88_BitFieldSet(KSZ_ISR, ISR_RXIS);
        ethData.pktReady = true;
    }
    // KSZ88_RegWrite16(KSZ_IER, IER_LCIE | IER_TXIE | IER_RXIE | IER_LDIE);
    PRINTF("%s>", __FUNCTION__);;
}

/**
 * Retrieve the packet and write it to software buffer
 */
void ETH_NextPacketUpdate(void)
{
    PRINTF("<%s", __FUNCTION__);;
    uint8_t rxfc_val = KSZ88_RegRead8(KSZ_RXFC);
    if (rxfc_val==0) return;
    // Disabling all interrupts before reading a packet
    // KSZ88_RegWrite16(KSZ_IER, 0x0000);
    uint16_t rxstat_val = KSZ88_RegRead16(KSZ_RXFHSR);
    rxstat_val = rxstat_val; // unused ?
    uint16_t rxlen_val = KSZ88_RegRead16(KSZ_RXFHBCR);
    rxlen_val = rxlen_val & 0xfff;
    
    //Initialize the RX buffer
    RXBuffer_Init();
    KSZ88_BitFieldClear(KSZ_RXFDPR, pointerMask);
    KSZ88_BitFieldSet(KSZ_RXQCR, RXQCR_SDA);
    ETH_NCS_LOW();
    ETH_SPI_WRITE8(fifo_read);
    ETH_SPI_READ8();
    ETH_SPI_READ8();
    ETH_SPI_READ8();
    ETH_SPI_READ8();
    ((char *) &rxPacketStatusVector)[0] = ETH_SPI_READ8();
    ((char *) &rxPacketStatusVector)[1] = ETH_SPI_READ8();
    ((char *) &rxPacketStatusVector)[2] = ETH_SPI_READ8();
    ((char *) &rxPacketStatusVector)[3] = ETH_SPI_READ8();
    ETH_SPI_READ8();
    ETH_SPI_READ8();

    // Received frame length = RXFHBCR minus 4-byte CRC and 2-byte offset enable
    BufferReceive(&dataBuffers.rxBuffer, (rxlen_val-6));
    ETH_NCS_HIGH();
    KSZ88_BitFieldClear(KSZ_RXQCR, RXQCR_SDA);
    // Enabling the interrupts after reading a packet from RX FIFO
    // KSZ88_RegWrite16(KSZ_IER, IER_LCIE | IER_TXIE | IER_RXIE | IER_LDIE);
    PRINTF("%s> status: %04X, cnt: %u", __FUNCTION__, rxPacketStatusVector.rxfhsr, rxPacketStatusVector.rxfhbcr);;
}

/**
 * Start a packet if the Ethernet transmitter is idle.
 * @param dest_mac
 * @param type
 * @return SUCCESS if packet started. BUFFER_BUSY or TX_LOGIC_NOT_IDLE if buffer or transmitter is busy respectively
 */
error_msg ETH_WriteStart(const mac48Address_t *dest_mac, uint16_t type)
{
    PRINTF("<%s", __FUNCTION__);;
    //Initialize the TX buffer
    TXBuffer_Init();

    // Check for TX in progress
    if((KSZ88_RegRead16(KSZ_TXQCR) & TXQCR_METFE))
        return TX_LOGIC_NOT_IDLE;

    if( KSZ88_RegRead16(KSZ_TXMIR) < TX_BUFFER_MID)
        return BUFFER_BUSY;

    // Initialize a new packet handler. It is automatically placed in the queue
    txPacket_t* ethPacket = (txPacket_t*)ETH_NewPacket();

    if( ethPacket == NULL ) // No more available packets
    {
        PRINTF("ERROR: %s - no more packets available!", __FUNCTION__);
        return BUFFER_BUSY;
    }

    SetBit(ethPacket->flags, ETH_WRITE_IN_PROGRESS);    // writeInProgress = true;

    ETH_ResetByteCount(); 

    // Disabling all interrupts before starting a packet
    // KSZ88_RegWrite16(KSZ_IER, 0x0000);

    BufferWrite16(&dataBuffers.txBuffer, controlWord);
    BufferWrite16(&dataBuffers.txBuffer, packetLength);
    BufferWriteBlock(&dataBuffers.txBuffer, (char*)dest_mac->array, 6);
    BufferWriteBlock(&dataBuffers.txBuffer, (char*)macAddress.array, 6);
    BufferWrite16(&dataBuffers.txBuffer, type);

    PRINTF("%s>", __FUNCTION__);;
    return SUCCESS;
}

/**
 * Enqueue the latest written packet and start the transmission of a queued packet
 * @return
 */
static error_msg ETH_SendQueued(void)
{
    PRINTF("<%s", __FUNCTION__);
    error_msg msg = ERROR;
    if ( pHead->flags & ETH_TX_QUEUED )
    {
        // "Close" the latest written packet and enqueue it
        // txQueued = false
        ClearBit( pHead->flags, ETH_TX_QUEUED);         

        //Start sending
        KSZ88_BitFieldSet(KSZ_TXQCR, TXQCR_METFE);   
        while(KSZ88_RegRead16(KSZ_TXQCR) & TXQCR_METFE);
        ETH_RemovePacket(pTail);
        msg = SUCCESS;
    }
    else
        msg = BUFFER_BUSY;
    PRINTF("%s> : %u", __FUNCTION__, msg);
    return msg;
}

/**
 * Start the Transmission
 * @return
 */
error_msg ETH_Send(void)
{
    PRINTF("<%s..", __FUNCTION__);
    KSZ88_RegRead16(KSZ_ISR);
    KSZ88_BitFieldSet(KSZ_RXQCR, RXQCR_SDA);
    ETH_NCS_LOW();
    ETH_SPI_WRITE8(fifo_write);
    BufferSend(&dataBuffers.txBuffer);
    ETH_NCS_HIGH();
    KSZ88_BitFieldClear(KSZ_RXQCR, RXQCR_SDA);
    while(KSZ88_RegRead16(KSZ_RXQCR)==0x0238)
        delay(1000);
    delay(100);
    KSZ88_RegRead16(KSZ_ISR);
    
    // Enabling the interrupts after sending a packet
    // KSZ88_RegWrite16(KSZ_IER, IER_LCIE | IER_TXIE | IER_RXIE | IER_LDIE);
    
    pHead->packetEnd = ((KSZ88_RegRead16(KSZ_TXFDPR)) & pointerMask) - 1;
    
    if (ethData.up == 0)
        return LINK_NOT_FOUND;

    if( ethListSize == 0 )
        return BUFFER_BUSY;
    
    // writeInProgress = false
    ClearBit( pHead->flags, ETH_WRITE_IN_PROGRESS);     
    
    // txQueued = true
    SetBit( pHead->flags, ETH_TX_QUEUED);               
   
    // The packet is prepared to be sent / queued at this time
    if( (KSZ88_RegRead16(KSZ_TXQCR) & TXQCR_METFE) || (ethListSize > 1) )
        return TX_QUEUED;

    return ETH_SendQueued();
}

/**
 * Reset RX part
 */
void ETH_ResetReceiver(void)
{
    PRINTF("<%s>", __FUNCTION__);
    KSZ88_BitFieldClear(KSZ_RXCR1, RXCR1_RXE);
    KSZ88_BitFieldSet(KSZ_RXCR1, RXCR1_FRXQ);
    KSZ88_BitFieldClear(KSZ_RXCR1, RXCR1_FRXQ);
    KSZ88_BitFieldSet(KSZ_RXCR1, RXCR1_RXE);
}

/**
 * Clears all bytes from the RX buffer
 */
void ETH_Flush(void)
{
    PRINTF("<%s..", __FUNCTION__);
    ethData.pktReady = false;
    ETH_ResetReceiver();  
}

/**
 * Calculate the TX Checksum - Software Checksum
 * @param position
 * @param length
 * @param seed
 * @return
 */
uint16_t ETH_TxComputeChecksum(uint16_t position, uint16_t length, uint16_t seed)
{
    PRINTF("<%s> pos: %u, len: %u, seed: %u", __FUNCTION__, position, length, seed);
    uint16_t value;

    uint32_t checksum = seed;
    position += 4; // sizeof(controlWord) + sizeof(packetLength);
    for(uint16_t i=0; i<length; i++)
    {
        while(length > 1)
        {
            ((char *)&value)[1] = BufferPeekByte( &dataBuffers.txBuffer, position++);
            ((char *)&value)[0] = BufferPeekByte( &dataBuffers.txBuffer, position++);

            checksum += value;
            length -= 2;
        }
        if(length!=0)
        {
            ((char *)&value)[1] = BufferPeekByte( &dataBuffers.txBuffer, position);
            ((char *)&value)[0] = 0;
            checksum += value;
        }
    }

    // wrap the checksum if its greater than 16 bits
    while(checksum >> 16)
        checksum = ((checksum & 0xFFFF) + (checksum>>16));

    // invert the number.
    checksum = ~checksum;
    checksum = (uint16_t)checksum;
   
    checksum = htons(checksum);
    return (uint16_t)checksum;
}

/**
 * Calculate RX checksum - Software checksum
 * @param len
 * @param seed
 * @return
 */
uint16_t ETH_RxComputeChecksum(uint16_t len, uint16_t seed)
{
    PRINTF("<%s> len: %u, seed: %u", __FUNCTION__, len, seed);
    uint16_t value;
    uint16_t offset = 0;
       
    uint32_t checksum = seed;
    
    for(uint16_t i=0; i<len; i++)
    {   
        while(len > 1)
        {
            // value = 0;
            ((char *)&value)[1] = BufferPeekByte( &dataBuffers.rxBuffer, offset++);
            ((char *)&value)[0] = BufferPeekByte( &dataBuffers.rxBuffer, offset++);
            checksum += value;
            len -= 2;
        }
        if(len)
        {
            // value = 0;
            ((char *)&value)[1] = BufferPeekByte( &dataBuffers.rxBuffer, offset);
            ((char *)&value)[0] = 0;
            checksum += value;
        }
    }
    // wrap the checksum if its greater than 16 bits
    while(checksum >> 16)
        checksum = ((checksum & 0xFFFF) + (checksum>>16));

    // invert the number.
    checksum = ~checksum;
    checksum = (uint16_t)checksum;

    // Return the resulting checksum
    return (uint16_t)((checksum & 0xFF00) >> 8) | ((checksum & 0x00FF) << 8);
}

/**
 * To get the MAC address
 * @param mac
 */
void ETH_GetMAC(uint8_t *macAddr)
{
    uint16_t addrH = KSZ88_RegRead16(KSZ_MARH);
    *macAddr++ = (addrH >> 8);
    *macAddr++ = (addrH & 0xFF);
    uint16_t addrM = KSZ88_RegRead16(KSZ_MARM);
    *macAddr++ = (addrM >> 8);
    *macAddr++ = (addrM & 0xFF);
    uint16_t addrL = KSZ88_RegRead16(KSZ_MARL);
    *macAddr++ = (addrL >> 8);
    *macAddr++ = (addrL & 0xFF);
    PRINTF("<%s> %s", __FUNCTION__, printMAC(macAddr-6));
}

/**
 * To set the MAC address
 * @param mac
 */
void ETH_SetMAC(uint8_t *macAddr)
{
    PRINTF("<%s> %s", __FUNCTION__, printMAC(macAddr));
    KSZ88_RegWrite16(KSZ_MARH, (macAddr[0] << 8) | macAddr[1]);
    KSZ88_RegWrite16(KSZ_MARM, (macAddr[2] << 8) | macAddr[3]);
    KSZ88_RegWrite16(KSZ_MARL, (macAddr[4] << 8) | macAddr[5]);     
}

void ETH_SaveRDPT(void)
{
    ethData.saveRDPT = (uint32_t)(dataBuffers.rxBuffer.start - dataBuffers.rxBuffer.readPointer);
    PRINTF("<%s> : %08X", __FUNCTION__, ethData.saveRDPT);
}

uint32_t ETH_GetReadPtr(void)
{
    uint32_t val = (uint32_t)dataBuffers.rxBuffer.readPointer;
    PRINTF("<%s> : %08X", __FUNCTION__, val);
    return val;
}

extern "C" {
void ETH_SetReadPtr(uint32_t rdptr)
{
    PRINTF("<%s> : %08X", __FUNCTION__, rdptr);
    dataBuffers.rxBuffer.readPointer = (uint8_t*)rdptr;
}
}

void ETH_ResetByteCount(void)
{
    ethData.saveWRPT = dataBuffers.txBuffer.dataLength;
    PRINTF("<%s> : %08X", __FUNCTION__, ethData.saveWRPT);
}

uint16_t ETH_GetByteCount(void)
{
    uint16_t ret = (uint16_t)(dataBuffers.txBuffer.dataLength - ethData.saveWRPT);
    PRINTF("<%s> : %u", __FUNCTION__, ret);
    return ret;
}

