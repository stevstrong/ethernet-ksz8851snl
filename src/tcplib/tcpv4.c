/**
 * Transmission Control Protocol version4 (TCPv4) Implementation Source File
 *
 * @file tcpv4.c
 *
 * @ingroup tcp
 *
 * @brief This file provides the API implementation for the TCPv4 stack.
 *
 * @version TCP/IP Lite Driver Version 5.0.0
 */
/*
    © [2023] Microchip Technology Inc. and its subsidiaries

    Subject to your compliance with these terms, you may use Microchip software and any derivatives 
    exclusively with Microchip products. You are responsible for complying with 3rd party license terms 
    applicable to your use of 3rd party software (including open source software) that may accompany 
    Microchip software. SOFTWARE IS “AS IS.” NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR 
    STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED WARRANTIES OF NON-
    INFRINGEMENT, MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT 
    WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR 
    CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED 
    TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE 
    POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE FULLEST EXTENT ALLOWED BY 
    LAW, MICROCHIP’S TOTAL LIABILITY ON ALL CLAIMS RELATED TO THE SOFTWARE WILL NOT 
    EXCEED AMOUNT OF FEES, IF ANY, YOU PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
    */

// Included Files
#include <stdio.h>
#include <stddef.h>
#include <string.h>
#include "ipv4.h"
#include "tcpv4.h"
#include "network.h"
#include "../ethernet_driver.h"
#include "tcpip_types.h"
#include "log.h"
#include "tcpip_config.h"

// Variables
tcpTCB_t *tcbList;
socklistsize_t tcbListSize;
tcpTCB_t *currentTCB;

static tcpHeader_t tcpHeader;
static uint16_t nextAvailablePort;
static uint32_t nextSequenceNumber;

static uint32_t receivedRemoteAddress;
static uint16_t rcvPayloadLen;
static uint16_t tcpMss = 536;

static uint16_t tcpDataLength;
static uint16_t bytesToSendForRetransmit = 0;
static uint8_t *txBufferPtrForRetransmit;
static uint32_t localSeqnoForRetransmit;
static uint32_t lastAckNumber;

#ifdef ENABLE_NETWORK_DEBUG
#define logMsg(msg, msgSeverity, msgLogDest) logMessage(msg, LOG_KERN, msgSeverity, msgLogDest)
#else
#define logMsg(msg, msgSeverity, msgLogDest)
#endif

// Local TCP Public Interface Function Prototypes
/**
 * @ingroup tcp
 * @brief TCP stack state machine called upon the occurrence of each event
 * (e.g., opening/closing a connection, receiving a TCP packet).
 * @param None.
 * @return Error status. Refer to the error description in tcpip_types.h.
 */
static error_msg TCP_FiniteStateMachine(void);

/**
 * @ingroup tcp
 * @brief Retransmits TCP packets after timeout
 * @param None.
 * @return Error status. Refer to the error description in tcpip_types.h.
 */
static error_msg TCP_TimoutRetransmit(void);

/**
 * @ingroup tcp
 * @brief Inserts a pointer to the new TCB into the TCB pointer list.
 * @param *ptr Pointer to the user-allocated memory for the TCB structure
 * @return None.
 */
static void TCB_Insert(tcpTCB_t *ptr);

/**
 * @ingroup tcp
 * @brief Removes a pointer to a TCB from the TCB pointer list and connects its adjacent items together.
 * @param *ptr Pointer to the user-allocated memory for the TCB structure
 * @return None.
 */
static void TCB_Remove(tcpTCB_t *ptr);

/**
 * @ingroup tcp
 * @brief Resets the socket to a known state.
 * @param *tcbPtr Pointer to the socket/TCB structure
 * @return None.
 */
static void TCB_Reset(tcpTCB_t *tcbPtr);

/**
 * @ingroup tcp
 * @brief Check if there is a pointer to a socket/TCB. If the pointer is in the TCB list, then it is a valid socket.
 * @param *ptr Pointer to the socket/TCB structure
 * @return Error status. Refer to the error description in tcpip_types.h.
 */
static error_msg TCB_Check(tcpTCB_t *ptr);

/**
 * @ingroup tcp
 * @brief Internal function of the TCP stack to send a TCP packet.
 * @param *tcbPtr Pointer to the socket/TCB structure
 * @retval True Sending the buffer succeeded
 * @retval False Sending the buffer failed
 */
static error_msg TCP_Snd(tcpTCB_t *tcbPtr);

/**
 * @ingroup tcp
 * @brief Copies the TCP packet payload to the socket RX buffer.
 * This routine also sends the ACK for the received packet and any data ready to be sent.
 * @param len Length of the payload received
 * @retval True Copying the payload to the RX buffer succeeded
 * @retval False Copying the payload to the RX buffer failed
 */
error_msg TCP_PayloadSave(uint16_t len);

/**
 * @ingroup tcp
 * @brief Reads and parses the Options field in the TCP header.
 * This routine reads only the ones that have SYN or SYN + ACK.
 * For the other TCP headers the field will be skipped.
 * @param None.
 * @retval True Parsing the Options field succeeded
 * @retval False Parsing the Options field failed
 */
static error_msg TCP_ParseTCPOptions(void);

/**
 * @ingroup tcp
 * @brief Identifies the destination socket and parses the TCP header.
 * This routine is called by the IP layer for each received TCP packet.
 * @param remoteAddress Source IP address for the received TCP packet
 * @param length Length of the TCP payload
 * @return None.
 */
void TCP_Recv(uint32_t remoteAddress, uint16_t length);

// TCP Public Interface Function Definitions
static void TCB_Insert(tcpTCB_t *ptr)
{
    PRINTF("<%s>", __FUNCTION__);
    // Insert the new TCB at the head of the list.
    // This prevents a list traversal and saves time.
    if (tcbList != NULL)
    {
        // Link this TCB as the previous one for the top of the list
        tcbList->prevTCB = ptr;
    }
    ptr->nextTCB = tcbList; // Put the existing list at the end of this tcb.
    tcbList = ptr;          // Put this tcb at the head of the list.
    ptr->prevTCB = NULL;    // Make sure that the upstream pointer is empty.
    tcbListSize++;
}

static void TCB_Remove(tcpTCB_t *ptr)
{
    PRINTF("<%s>", __FUNCTION__);
    if (tcbListSize > 1)
    {
        // check if this is the first in list
        if (ptr->prevTCB == NULL)
        {
            tcbList = ptr->nextTCB;
            ((tcpTCB_t *)(ptr->nextTCB))->prevTCB = NULL;
        }
        else
        {
            ((tcpTCB_t *)(ptr->prevTCB))->nextTCB = ptr->nextTCB;
            ((tcpTCB_t *)(ptr->nextTCB))->prevTCB = ptr->prevTCB;
        }
        tcbListSize--;
    }
    else if (tcbListSize == 1)
    {
        tcbList = NULL;
    }
}

static void TCB_Reset(tcpTCB_t *tcbPtr)
{
    PRINTF("<%s>", __FUNCTION__);
    tcbPtr->destIP              = 0;
    tcbPtr->destPort            = 0;
    tcbPtr->localSeqno          = 0;
    tcbPtr->localLastAck        = 0;
    tcbPtr->remoteSeqno         = 0;
    tcbPtr->remoteAck           = 0;
    tcbPtr->remoteWnd           = 0;

    tcbPtr->timeout             = 0;
    tcbPtr->timeoutReloadValue  = 0;
    tcbPtr->timeoutsCount       = 0;
    tcbPtr->flags               = 0;

    tcbPtr->localPort           = 0;
    tcbPtr->bytesSent           = 0;
    tcbPtr->payloadSave         = false;
    tcbPtr->socketState         = SOCKET_CLOSING;
}

static error_msg TCB_Check(tcpTCB_t *ptr)
{
    PRINTF("<%s>", __FUNCTION__);
    error_msg ret = ERROR;
    socklistsize_t count = 0;

    if ((tcbList != NULL) && (ptr != NULL))
    {
        // Search for ptr into the active TCB/sockets list
        tcpTCB_t *tcbPtr = tcbList;
        while ((tcbPtr != NULL) && (count < tcbListSize))
        {
            if (tcbPtr == ptr)
            {
                ret = SUCCESS;
                break;
            }
            else
            {
                tcbPtr = tcbPtr->nextTCB;
                count++;
            }
        }
    }
    return ret;
}

static error_msg TCP_Snd(tcpTCB_t *tcbPtr)
{
    // logMsg("TCP_Snd", LOG_INFO, LOG_DEST_CONSOLE);
    PRINTF("<%s", __FUNCTION__);
    error_msg ret = ERROR;
    uint8_t *data = NULL;

    tcpHeader_t txHeader;
    txHeader.sourcePort = htons(tcbPtr->localPort);
    txHeader.destPort = htons(tcbPtr->destPort);

    txHeader.sequenceNumber = htonl(tcbPtr->localSeqno);

    txHeader.ackNumber = htonl(tcbPtr->remoteAck); // Ask for next packet

    txHeader.ns = 0;         // Make sure to clean unused fields
    txHeader.reserved = 0;   // Make sure to clean unused fields
    txHeader.dataOffset = 5; // Support for options is not avaialble for now
    txHeader.windowSize = htons(tcbPtr->localWnd);
    txHeader.checksum = 0;
    txHeader.urgentPtr = 0;

    if ((tcbPtr->flags) & (TCP_SYN_FLAG | TCP_RST_FLAG))
    {
        tcpDataLength = 0; // SYN and RST packets doesn't have any payload
    }
    else if (tcbPtr->payloadSave == true)
    {
        tcpDataLength = 0;
    }
    else
    {
        tcpDataLength = tcbPtr->bytesSent;

        if (tcpDataLength != 0)
        {
            if (tcbPtr->remoteWnd == 0)
            {
                tcbPtr->remoteWnd = 1;
            }
            if (tcpDataLength > tcbPtr->remoteWnd)
            {
                tcpDataLength = tcbPtr->remoteWnd;
            }

            if (tcpDataLength > tcbPtr->mss)
            {
                tcpDataLength = tcbPtr->mss;
            }
            data = tcbPtr->txBufferPtr;

            // Update the pointer to the next byte that needs to be sent
            tcbPtr->txBufferPtr = tcbPtr->txBufferPtr + tcpDataLength;
            tcbPtr->bytesToSend = tcbPtr->bytesSent - tcpDataLength;

            if (tcbPtr->bytesToSend == 0)
            {
                tcbPtr->flags = tcbPtr->flags | TCP_PSH_FLAG;
            }
        }
    }
    // Update the TCP Flags
    txHeader.flags = tcbPtr->flags;
    uint16_t payloadLength = sizeof(tcpHeader_t) + tcpDataLength;

    ret = IPv4_Start(tcbPtr->destIP, TCP_TCPIP);
    if (ret == SUCCESS)
    {
        ETH_WriteBlock((char *)&txHeader, sizeof(tcpHeader_t));

        if (data && tcpDataLength > 0)
        {
            ETH_WriteBlock((char *)data, tcpDataLength);
        }

        uint16_t cksm = payloadLength + TCP_TCPIP;
        // Calculate the TCP checksum
        cksm = ETH_TxComputeChecksum(sizeof(ethernetFrame_t) + sizeof(ipv4Header_t) - 8, payloadLength + 8, cksm);
        ETH_Insert((char *)&cksm, 2, sizeof(ethernetFrame_t) + sizeof(ipv4Header_t) + offsetof(tcpHeader_t, checksum));

        ret = IPV4_Send(payloadLength);
        // tcbPtr->txBufferPtr = tcbPtr->txBufferPtr - tcpDataLength;
    }

    // The packet wasn't transmitted
    // Use the timeout to retry again later
    if (ret != SUCCESS && ret != TX_QUEUED)
    {
        // Making sure to keep the remaining timeouts and skip this send that failed
        // Trying at least once
        tcbPtr->timeoutsCount = tcbPtr->timeoutsCount - 1u;

        if (tcbPtr->timeout == 0)
        {
            tcbPtr->timeout = TCP_START_TIMEOUT_VAL;
        }
    }
    else
    {
        // Incrementing the sequence number if the packet is sent
        tcbPtr->localSeqno = tcbPtr->localSeqno + tcpDataLength;
        logMsg("tcp_packet sent", LOG_INFO, LOG_DEST_CONSOLE);
    }
    PRINTF("%s>", __FUNCTION__);
    return ret;
}

error_msg TCP_PayloadSave(uint16_t len)
{
    logMsg("TCP_ParseTCPOptions", LOG_INFO, LOG_DEST_CONSOLE);
    error_msg ret = ERROR;
    uint16_t buffer_size;

    // Check if there is a valid buffer
    if (currentTCB->rxBufState == RX_BUFF_IN_USE)
    {
        // Make sure there is enough space
        if (currentTCB->localWnd >= len)
        {
            buffer_size = len;
        }
        else
        {
            buffer_size = currentTCB->localWnd;
        }

        ETH_ReadBlock((char*)currentTCB->rxBufferPtr, buffer_size);
        currentTCB->rxBufferPtr = currentTCB->rxBufferPtr + buffer_size;

        // Updating the local window to inform the remote of the available space
        currentTCB->localWnd  = currentTCB->localWnd - buffer_size;
        currentTCB->remoteAck = currentTCB->remoteSeqno + buffer_size;

        // Prepareing to send the ACK and maybe some data if there are any
        currentTCB->flags = TCP_ACK_FLAG;
        currentTCB->payloadSave = true;

        TCP_Snd(currentTCB);
        currentTCB->payloadSave = false;
        ret = SUCCESS;
    }
    return ret;
}

static error_msg TCP_ParseTCPOptions(void)
{
    logMsg("TCP_ParseTCPOptions", LOG_INFO, LOG_DEST_CONSOLE);
    error_msg ret = ERROR;

    // Check for the option fields in TCP header
    uint16_t tcpOptionsSize = (uint16_t)(tcpHeader.dataOffset << 2u) - (uint16_t)sizeof(tcpHeader_t);

    if (tcpOptionsSize > 0)
    {
        // RFC 1122, page 85, Section 4.2.2.6  Maximum Segment Size Option: RFC-793 Section 3.1
        // More explanations can be found in RFC-6691
        tcpMss = 536;
        // Parse the option only for SYN segments
        if (tcpHeader.syn)
        {
            // Parse for the TCP MSS option, if present.
            while (tcpOptionsSize--)
            {
                uint8_t opt = ETH_Read8();
                switch (opt)
                {
                case TCP_EOP:
                    // End of options
                    if (tcpOptionsSize)
                    {
                        // Dumping remaining unused bytes
                        ETH_Dump(tcpOptionsSize);
                        tcpOptionsSize = 0;
                    }
                    ret = SUCCESS;
                    break;
                case TCP_NOP:
                    // NOP option.
                    break;
                case TCP_MSS:
                    if (tcpOptionsSize >= 3) // at least 3 more bytes
                    {
                        opt = ETH_Read8();
                        if (opt == 0x04)
                        {
                            // An MSS option with the right option length.
                            tcpMss = ETH_Read16(); // value returned in host endianess
                            // Advance to the next option
                            tcpOptionsSize = tcpOptionsSize - 3;

                            // Limit the mss to the configured TCP_MAX_SEG_SIZE
                            if (tcpMss > TCP_MAX_SEG_SIZE)
                            {
                                tcpMss = TCP_MAX_SEG_SIZE;
                            }
                            ret = SUCCESS;
                        }
                        else
                        {
                            // Bad option size length
                            logMsg("tcp_parseopt: bad option size length", LOG_INFO, LOG_DEST_CONSOLE);
                            // Unexpected error
                            tcpOptionsSize = 0;
                            ret = ERROR;
                        }
                    }
                    else
                    {
                        // Unexpected error
                        tcpOptionsSize = 0;
                        ret = ERROR;
                    }
                    break;
                default:
                    logMsg("tcp_parseopt: other", LOG_INFO, LOG_DEST_CONSOLE);
                    opt = ETH_Read8();
                    tcpOptionsSize--;

                    if (opt > 1) // This should be at least 2 to be valid
                    {
                        // Adjust for the remaining bytes for the current option
                        opt = opt - 2u;
                        if (opt <= tcpOptionsSize)
                        {
                            // All other options have a length field, so that it can easily be skiped
                            ETH_Dump(opt);
                            tcpOptionsSize = tcpOptionsSize - opt;
                            ret = SUCCESS;
                        }
                        else
                        {
                            logMsg("tcp_parseopt: bad option length", LOG_INFO, LOG_DEST_CONSOLE);
                            // The options are malformed and will not be processed further
                            tcpOptionsSize = 0;
                            ret = ERROR;
                        }
                    }
                    else
                    {
                        logMsg("tcp_parseopt: bad length", LOG_INFO, LOG_DEST_CONSOLE);
                        // If the length field is zero, the options are malformed
                        // and will not be processed further
                        tcpOptionsSize = 0;
                        ret = ERROR;
                    }
                    break;
                }
            }
        }
        else // jump over the Options from TCP header
        {
            ETH_Dump(tcpOptionsSize);
            ret = SUCCESS;
        }
    }
    else
    {
        ret = SUCCESS;
    }

    return ret;
}

void TCP_Recv(uint32_t remoteAddress, uint16_t length)
{
    logMsg("TCP_Recv", LOG_INFO, LOG_DEST_CONSOLE);

    socklistsize_t count = 0;
    // Make sure to not reuse old values
    receivedRemoteAddress = 0;
    rcvPayloadLen = 0;

    ETH_ReadBlock((char *)&tcpHeader, sizeof(tcpHeader_t));

    currentTCB = NULL;

    // Quick check on destination port
    if ((tcpHeader.destPort != 0) && (tcpHeader.sourcePort != 0))
    {
        tcpHeader.sourcePort = ntohs(tcpHeader.sourcePort);
        tcpHeader.destPort = ntohs(tcpHeader.destPort);

        // Search for active TCB
        tcpTCB_t *tcbPtr = tcbList;
        while ((tcbPtr != NULL) && (count < tcbListSize))
        {
            if (tcpHeader.destPort == tcbPtr->localPort)
            {
                currentTCB = tcbPtr;
                break;
            }
            else
            {
                tcbPtr = tcbPtr->nextTCB;
                count++;
            }
        }

        if (currentTCB != NULL)
        {
            if ((tcpHeader.sourcePort == currentTCB->destPort) ||
                (currentTCB->destIP == 0))
            {
                // Need this if the port is in listen mode
                // or to check for the correct TCB
                receivedRemoteAddress = remoteAddress;
                rcvPayloadLen = length - (uint16_t)(tcpHeader.dataOffset << 2);

                // check/skip the TCP header options
                if (TCP_ParseTCPOptions() == SUCCESS)
                {
                    // A packet was received
                    // sort out the events
                    if (tcpHeader.syn)
                    {
                        if (tcpHeader.ack)
                        {
                            logMsg("found syn&ack", LOG_INFO, LOG_DEST_CONSOLE);
                            currentTCB->connectionEvent = RCV_SYNACK;
                        }
                        else
                        {
                            logMsg("found syn", LOG_INFO, LOG_DEST_CONSOLE);
                            currentTCB->connectionEvent = RCV_SYN;
                        }
                    }
                    else if (tcpHeader.fin)
                    {
                        if (tcpHeader.ack)
                        {
                            logMsg("found fin&ack", LOG_INFO, LOG_DEST_CONSOLE);
                            currentTCB->connectionEvent = RCV_FINACK;
                        }
                        else
                        {
                            logMsg("found fin", LOG_INFO, LOG_DEST_CONSOLE);
                            currentTCB->connectionEvent = RCV_FIN;
                        }
                    }
                    else if (tcpHeader.rst)
                    {
                        if (tcpHeader.ack)
                        {
                            logMsg("found rst&ack", LOG_INFO, LOG_DEST_CONSOLE);
                            currentTCB->connectionEvent = RCV_RSTACK;
                        }
                        else
                        {
                            logMsg("found rst", LOG_INFO, LOG_DEST_CONSOLE);
                            currentTCB->connectionEvent = RCV_RST;
                        }
                    }
                    else if (tcpHeader.ack)
                    {
                        logMsg("found ack", LOG_INFO, LOG_DEST_CONSOLE);
                        currentTCB->connectionEvent = RCV_ACK;
                    }
                    else
                    {
                        logMsg("confused", LOG_INFO, LOG_DEST_CONSOLE);
                    }
                    // Conversion done herer save some cycles later
                    tcpHeader.ackNumber = ntohl(tcpHeader.ackNumber);
                    tcpHeader.sequenceNumber = ntohl(tcpHeader.sequenceNumber);

                    TCP_FiniteStateMachine();
                }
                else
                {
                    logMsg("pkt dropped: bad options", LOG_INFO, LOG_DEST_CONSOLE);
                }
            } // No reset message sent for PORT not open
        }
    }
}

static error_msg TCP_FiniteStateMachine(void)
{
    logMsg("TCP_FiniteStateMachine", LOG_INFO, LOG_DEST_CONSOLE);
    uint16_t notAckBytes;
    error_msg ret = ERROR;

    tcp_fsm_states_t nextState = currentTCB->fsmState; // Default don't change states
    tcpEvent_t event = currentTCB->connectionEvent;
    switch (currentTCB->fsmState)
    {
        case LISTEN:
            switch (event)
            {
                case RCV_SYN:
                    logMsg("LISTEN: rx_syn", LOG_INFO, LOG_DEST_CONSOLE);
                    // Start the connection on the TCB

                    currentTCB->destIP = receivedRemoteAddress;
                    currentTCB->destPort = tcpHeader.sourcePort;

                    currentTCB->localLastAck = 0;

                    currentTCB->remoteSeqno = tcpHeader.sequenceNumber;
                    currentTCB->remoteAck = currentTCB->remoteSeqno + 1; // Ask for next packet

                    // Save data from TCP header
                    currentTCB->remoteWnd = ntohs(tcpHeader.windowSize);
                    currentTCB->mss = tcpMss;

                    // Create and send a SYN+ACK packet
                    currentTCB->flags = TCP_SYN_FLAG | TCP_ACK_FLAG;
                    currentTCB->timeout = TCP_START_TIMEOUT_VAL;
                    currentTCB->timeoutReloadValue = TCP_START_TIMEOUT_VAL;
                    currentTCB->timeoutsCount = TCP_MAX_SYN_RETRIES;

                    TCP_Snd(currentTCB);
                    nextState = SYN_RECEIVED;
                    break;
                case CLOSE:
                    logMsg("LISTEN: close", LOG_INFO, LOG_DEST_CONSOLE);
                    nextState = CLOSED;
                    TCB_Reset(currentTCB);
                    break;
                default:
                    // For all other cases the packet is invalid and will be discarded
                    break;
            }
            break;
        case SYN_SENT:
            switch (event)
            {
                case RCV_SYN:
                    logMsg("SYN_SENT: rx_syn", LOG_INFO, LOG_DEST_CONSOLE);
                    // Simultaneous open
                    currentTCB->remoteSeqno = tcpHeader.sequenceNumber;
                    currentTCB->remoteAck = tcpHeader.sequenceNumber + 1; // Ask for next packet

                    // Save data from TCP header
                    currentTCB->remoteWnd = ntohs(tcpHeader.windowSize);
                    currentTCB->mss = tcpMss;

                    // Create and send a ACK packet
                    currentTCB->timeout = TCP_START_TIMEOUT_VAL;
                    currentTCB->timeoutReloadValue = TCP_START_TIMEOUT_VAL;
                    currentTCB->timeoutsCount = TCP_MAX_SYN_RETRIES;
                    currentTCB->flags = TCP_SYN_FLAG | TCP_ACK_FLAG;

                    TCP_Snd(currentTCB);

                    // Go to SYN_RECEIVED and waiting for the ack
                    nextState = SYN_RECEIVED;
                    break;
                case RCV_SYNACK:
                    logMsg("SYN_SENT: rx_synack", LOG_INFO, LOG_DEST_CONSOLE);

                    currentTCB->timeout = 0;

                    if ((currentTCB->localSeqno + 1) == tcpHeader.ackNumber)
                    {
                        // Create and send a ACK packet
                        currentTCB->localSeqno = currentTCB->localSeqno + 1;
                        currentTCB->flags = TCP_ACK_FLAG;

                        // Save data from TCP header
                        currentTCB->remoteSeqno = tcpHeader.sequenceNumber;
                        // Ask for next packet
                        currentTCB->remoteAck = tcpHeader.sequenceNumber + 1;

                        currentTCB->remoteWnd = ntohs(tcpHeader.windowSize);
                        currentTCB->mss = tcpMss;

                        if (TCP_Snd(currentTCB) == (TX_QUEUED || SUCCESS))
                        {
                            nextState = ESTABLISHED;
                            currentTCB->socketState = SOCKET_CONNECTED;
                        }
                    }
                    else
                    {
                        // Send reset
                        currentTCB->localSeqno = tcpHeader.ackNumber;
                        currentTCB->flags = TCP_RST_FLAG | TCP_ACK_FLAG;
                        if (TCP_Snd(currentTCB) == (TX_QUEUED || SUCCESS))
                        {
                            nextState = CLOSED;
                            TCB_Reset(currentTCB);
                        }
                    }
                    break;
                case RCV_ACK:
                    logMsg("SYN_SENT: rx_ack", LOG_INFO, LOG_DEST_CONSOLE);

                    currentTCB->timeout = 0;

                    if ((currentTCB->localSeqno + 1) == tcpHeader.ackNumber)
                    {
                        // Create and send a ACK packet
                        currentTCB->localSeqno = currentTCB->localSeqno + 1;
                        currentTCB->flags = TCP_ACK_FLAG;

                        // Save data from TCP header
                        currentTCB->remoteSeqno = tcpHeader.sequenceNumber;
                        currentTCB->remoteAck = tcpHeader.sequenceNumber + 1; // ask for next packet

                        currentTCB->remoteWnd = ntohs(tcpHeader.windowSize);
                        currentTCB->mss = tcpMss;

                        nextState = ESTABLISHED;
                        currentTCB->socketState = SOCKET_CONNECTED;
                    }
                    else
                    {
                        // Send reset
                        currentTCB->localSeqno = tcpHeader.ackNumber;
                        currentTCB->flags = TCP_RST_FLAG;
                        if (TCP_Snd(currentTCB) == (TX_QUEUED || SUCCESS))
                        {
                            nextState = CLOSED;
                            TCB_Reset(currentTCB);
                        }
                    }
                    break;
                case CLOSE:
                    logMsg("SYN_SENT: close", LOG_INFO, LOG_DEST_CONSOLE);
                    // Go to CLOSED state
                    nextState = CLOSED;
                    TCB_Reset(currentTCB);
                    break;
                case TIMEOUT:
                    logMsg("SYN_SENT: timeout", LOG_INFO, LOG_DEST_CONSOLE);
                    // Looks like the the packet was lost
                    // Check inside the packet to see where to jump next
                    if (currentTCB->timeoutsCount)
                    {
                        if (TCP_Snd(currentTCB) == (TX_QUEUED || SUCCESS))
                        {
                            if (currentTCB->flags & TCP_RST_FLAG)
                            {
                                nextState = CLOSED;
                                TCB_Reset(currentTCB);
                            }
                            else if (currentTCB->flags & TCP_ACK_FLAG)
                            {
                                nextState = ESTABLISHED;
                                currentTCB->socketState = SOCKET_CONNECTED;
                            }
                        }
                    }
                    else
                    {
                        // Just reset the connection if there is no reply
                        currentTCB->flags = TCP_RST_FLAG;
                        if (TCP_Snd(currentTCB) == (TX_QUEUED || SUCCESS))
                        {
                            nextState = CLOSED;
                            TCB_Reset(currentTCB);
                        }
                    }
                    break;
                case RCV_RST:
                case RCV_RSTACK:
                    // Port seems not to be opened
                    nextState = CLOSED;
                    TCB_Reset(currentTCB);
                    break;
                default:
                    // For all other cases the packet is invalid and will be discarded
                    break;
            }
            break;
        case SYN_RECEIVED:
            switch (event)
            {
                case RCV_SYNACK:
                    logMsg("SYN_RECEIVED: rx_synack", LOG_INFO, LOG_DEST_CONSOLE);
                    if (currentTCB->localPort == tcpHeader.destPort)
                    {
                        // stop the current timeout
                        currentTCB->timeout = 0;

                        // This is part of simultaneous open
                        if ((currentTCB->destIP == receivedRemoteAddress) && (currentTCB->destPort == tcpHeader.sourcePort))
                            if ((currentTCB->localSeqno + 1) == tcpHeader.ackNumber)
                                nextState = ESTABLISHED;

                        currentTCB->socketState = SOCKET_CONNECTED;
                    }
                    break;
                case RCV_ACK:
                    logMsg("SYN_RECEIVED: rx_ack", LOG_INFO, LOG_DEST_CONSOLE);

                    // Checking if the packet is for the curent TCB
                    // Checking the remote IP address and remote port
                    if ((currentTCB->destIP == receivedRemoteAddress) && (currentTCB->destPort == tcpHeader.sourcePort))
                    {
                        // Checking the sequence numbers to verify if it was the packet that was asked for
                        if (currentTCB->remoteAck == tcpHeader.sequenceNumber)
                        {
                            // Is ACK OK?
                            if ((currentTCB->localSeqno + 1) == tcpHeader.ackNumber)
                            {
                                currentTCB->localSeqno = currentTCB->localSeqno + 1;
                                // Stop the current timeout
                                currentTCB->timeout = 0;

                                nextState = ESTABLISHED;
                                currentTCB->socketState = SOCKET_CONNECTED;
                            }
                        }
                    }
                    break;
                case CLOSE:
                    logMsg("SYN_RECEIVED: close", LOG_INFO, LOG_DEST_CONSOLE);
                    // Stop the current timeout
                    currentTCB->timeout = 0;
                    // Need to send FIN and go to the FIN_WAIT_1
                    currentTCB->flags = TCP_FIN_FLAG;
                    currentTCB->timeout = TCP_START_TIMEOUT_VAL;
                    currentTCB->timeoutReloadValue = TCP_START_TIMEOUT_VAL;
                    currentTCB->timeoutsCount = TCP_MAX_RETRIES;

                    nextState = FIN_WAIT_1;
                    TCP_Snd(currentTCB);
                    break;
                case RCV_RSTACK:
                case RCV_RST:
                    // Reset the connection
                    logMsg("SYN_RECEIVED:  rx_rst", LOG_INFO, LOG_DEST_CONSOLE);
                    // Check if the local port matches or else drop the packet
                    if (currentTCB->localPort == tcpHeader.destPort)
                    {
                        if (currentTCB->remoteAck == tcpHeader.sequenceNumber)
                        {
                            logMsg("rst seq OK", LOG_INFO, LOG_DEST_CONSOLE);
                            currentTCB->destIP = 0;
                            currentTCB->destPort = 0;
                            currentTCB->localSeqno = 0;
                            currentTCB->localLastAck = 0;
                            currentTCB->remoteSeqno = 0;
                            currentTCB->remoteAck = 0;
                            currentTCB->remoteWnd = 0;
                            // TCP_MAX_SEG_SIZE instead of 0
                            currentTCB->mss = TCP_MAX_SEG_SIZE;

                            nextState = LISTEN;
                        }
                    }
                    break;
                case TIMEOUT:
                    logMsg("SYN_RECEIVED:  timeout", LOG_INFO, LOG_DEST_CONSOLE);
                    if (currentTCB->timeoutsCount)
                    {
                        TCP_Snd(currentTCB);
                    }
                    else
                    {
                        // Reseting the connection if there is no reply
                        currentTCB->flags = TCP_RST_FLAG;
                        if (TCP_Snd(currentTCB) == (TX_QUEUED || SUCCESS))
                        {
                            currentTCB->destIP = 0;
                            currentTCB->destPort = 0;
                            currentTCB->localSeqno = 0;
                            currentTCB->localLastAck = 0;
                            currentTCB->remoteSeqno = 0;
                            currentTCB->remoteAck = 0;
                            currentTCB->remoteWnd = 0;
                            // TCP_MAX_SEG_SIZE instead of 0
                            currentTCB->mss = TCP_MAX_SEG_SIZE;
                            nextState = LISTEN;
                        }
                    }
                    break;
                default:
                    // Invalid packet so drop it
                    break;
            }
            break;
        case ESTABLISHED:
            switch (event)
            {
                case RCV_ACK:
                    logMsg("ESTABLISHED: rx_ack", LOG_INFO, LOG_DEST_CONSOLE);
                    if (currentTCB->destIP == receivedRemoteAddress)
                    {
                        // Is sequence number OK?
                        // Remote ACK should be equal to header sequence number
                        // Not accepting out of order packet (not enough memory)
                        if (currentTCB->remoteAck == tcpHeader.sequenceNumber)
                        {
                            // This is a ACK packet only
                            // Checking the ACK sequence
                            // Checking if this is already present in the received ACK
                            if (currentTCB->localLastAck < tcpHeader.ackNumber)
                            {
                                // Check how many bytes sent was acknowledged
                                if ((currentTCB->localSeqno + 1) >= tcpHeader.ackNumber)
                                {
                                    notAckBytes = (uint16_t)(currentTCB->localSeqno - tcpHeader.ackNumber);

                                    // Updating the pointer for next TX
                                    currentTCB->txBufferPtr = currentTCB->txBufferPtr - notAckBytes;
                                    currentTCB->bytesToSend = currentTCB->bytesToSend + notAckBytes;

                                    currentTCB->localLastAck = tcpHeader.ackNumber - 1;
                                    currentTCB->localSeqno = tcpHeader.ackNumber;
                                    if (bytesToSendForRetransmit == 0)
                                    {
                                        localSeqnoForRetransmit = currentTCB->localSeqno;
                                    }
                                    // Checking if all TX buffer/data was acknowledged
                                    if (currentTCB->bytesToSend == 0)
                                    {
                                        if (currentTCB->txBufState == TX_BUFF_IN_USE)
                                        {
                                            currentTCB->txBufState = NO_BUFF;
                                            // Stopping timeout
                                            currentTCB->timeout = 0;
                                        }
                                    }
                                    else
                                    {
                                        if (bytesToSendForRetransmit)
                                        {
                                            currentTCB->txBufferPtr = txBufferPtrForRetransmit;
                                            currentTCB->bytesSent = bytesToSendForRetransmit;
                                            currentTCB->localSeqno = localSeqnoForRetransmit;
                                        }
                                        else
                                        {
                                            currentTCB->bytesSent = currentTCB->bytesToSend;
                                        }
                                        currentTCB->timeoutReloadValue = TCP_START_TIMEOUT_VAL;
                                        currentTCB->timeoutsCount = TCP_MAX_RETRIES;
                                        TCP_Snd(currentTCB);
                                        if ((bytesToSendForRetransmit > 0) && (lastAckNumber != tcpHeader.ackNumber))
                                        {
                                            bytesToSendForRetransmit = 0;
                                        }
                                    }

                                    // Checking if the packet has payload
                                    if (rcvPayloadLen > 0)
                                    {
                                        currentTCB->remoteSeqno = tcpHeader.sequenceNumber;

                                        // Copying the payload to the local buffer
                                        TCP_PayloadSave(rcvPayloadLen);
                                    }
                                }
                                else
                                {
                                    // This is a wrong ACK
                                    // ACK a packet that wasn't transmitted
                                }
                            }
                        }
                    }
                    break;
                case CLOSE:
                    logMsg("ESTABLISHED: close", LOG_INFO, LOG_DEST_CONSOLE);
                    currentTCB->flags = TCP_FIN_FLAG | TCP_ACK_FLAG;
                    nextState = FIN_WAIT_1;
                    currentTCB->timeout = 0;
                    currentTCB->timeout = TCP_START_TIMEOUT_VAL;
                    currentTCB->timeoutReloadValue = TCP_START_TIMEOUT_VAL;
                    currentTCB->timeoutsCount = TCP_MAX_RETRIES;
                    TCP_Snd(currentTCB);
                    break;
                case RCV_FIN:
                    logMsg("ESTABLISHED: rx_fin", LOG_INFO, LOG_DEST_CONSOLE);
                    break;
                case RCV_FINACK:
                    if (currentTCB->destIP == receivedRemoteAddress)
                    {
                        // Is sequence number OK?
                        // Remote ACK should be equal to header sequence number
                        // Not accepting out of order packet (not enough memory)
                        if (currentTCB->remoteAck == tcpHeader.sequenceNumber)
                        {
                            currentTCB->bytesSent = 0;
                            // ACK the current packet
                            currentTCB->localSeqno = tcpHeader.ackNumber;
                            currentTCB->remoteAck = currentTCB->remoteAck + 1;

                            // Check if the packet has payload added
                            if (rcvPayloadLen > 0)
                            {
                                currentTCB->remoteSeqno = tcpHeader.sequenceNumber;

                                // Copy the payload to the local buffer
                                TCP_PayloadSave(rcvPayloadLen);
                            }

                            currentTCB->socketState = SOCKET_CLOSING;
                            currentTCB->timeout = TCP_START_TIMEOUT_VAL;
                            currentTCB->timeoutReloadValue = TCP_START_TIMEOUT_VAL;
                            currentTCB->timeoutsCount = TCP_MAX_RETRIES;
                            // Jump over CLOSE_WAIT state and send one packet with FIN + ACK
                            currentTCB->flags = TCP_FIN_FLAG | TCP_ACK_FLAG;

                            nextState = LAST_ACK;
                            TCP_Snd(currentTCB);
                        }
                    }
                    break;
                case RCV_RST:
                case RCV_RSTACK:
                    currentTCB->flags = TCP_RST_FLAG;
                    TCP_Snd(currentTCB);
                    nextState = CLOSED;
                    TCB_Reset(currentTCB);
                    break;
                case TIMEOUT:
                    logMsg("ESTABLISHED:  timeout", LOG_INFO, LOG_DEST_CONSOLE);
                    if (currentTCB->timeoutsCount)
                    {
                        TCP_TimoutRetransmit();
                    }
                    else
                    {
                        // Reset the connection if there is no reply
                        currentTCB->flags = TCP_RST_FLAG;
                        if (TCP_Snd(currentTCB) == (TX_QUEUED || SUCCESS))
                        {
                            nextState = CLOSED;
                            TCB_Reset(currentTCB);
                        }
                    }
                    break;
                default:
                    break;
            }
            break;
        case FIN_WAIT_1:
            switch (event)
            {
                case RCV_FIN:
                    currentTCB->flags = TCP_ACK_FLAG;
                    if (currentTCB->remoteAck == tcpHeader.sequenceNumber)
                    {
                        currentTCB->bytesSent = 0;
                        currentTCB->localSeqno = currentTCB->localSeqno + 1;
                        currentTCB->remoteAck = currentTCB->remoteAck + 1;
                        if (TCP_Snd(currentTCB) == (TX_QUEUED || SUCCESS))
                        {
                            nextState = CLOSING;
                        }
                    }
                    break;
                case RCV_ACK:
                    logMsg("FIN_WAIT_1: rx_ack", LOG_INFO, LOG_DEST_CONSOLE);
                    // Stop the current timeout
                    currentTCB->timeout = TCP_START_TIMEOUT_VAL;
                    currentTCB->timeoutsCount = 1;
                    nextState = FIN_WAIT_2;
                    break;
                case RCV_FINACK:
                    logMsg("FIN_WAIT_1: rx_finack", LOG_INFO, LOG_DEST_CONSOLE);
                    currentTCB->flags = TCP_ACK_FLAG;
                    if (currentTCB->remoteAck == tcpHeader.sequenceNumber)
                    {
                        currentTCB->bytesSent = 0;
                        currentTCB->localSeqno = currentTCB->localSeqno + 1;
                        currentTCB->remoteAck = currentTCB->remoteAck + 1;
                        if (TCP_Snd(currentTCB) == (TX_QUEUED || SUCCESS))
                        {
                            nextState = TIME_WAIT;
                        }
                    }
                    break;
                case TIMEOUT:
                    logMsg("FIN_WAIT_1:  timeout", LOG_INFO, LOG_DEST_CONSOLE);
                    if (currentTCB->timeoutsCount)
                    {
                        TCP_Snd(currentTCB);
                    }
                    else
                    {
                        // Just reset the connection if there is no reply
                        currentTCB->flags = TCP_RST_FLAG;
                        if (TCP_Snd(currentTCB) == (TX_QUEUED || SUCCESS))
                        {
                            nextState = CLOSED;
                            TCB_Reset(currentTCB);
                        }
                    }
                    break;
                default:
                    break;
            }
            break;
        case FIN_WAIT_2:
            switch (event)
            {
                case RCV_FINACK:
                case RCV_FIN:
                    logMsg("FIN_WAIT_2: rx_fin/rx_finack", LOG_INFO, LOG_DEST_CONSOLE);
                    currentTCB->flags = TCP_ACK_FLAG;
                    if (currentTCB->remoteAck == tcpHeader.sequenceNumber)
                    {
                        currentTCB->bytesSent = 0;
                        currentTCB->localSeqno = currentTCB->localSeqno + 1;
                        currentTCB->remoteAck = currentTCB->remoteAck + 1;
                        if (TCP_Snd(currentTCB) == (TX_QUEUED || SUCCESS))
                        {
                            nextState = TIME_WAIT;
                        }
                    }

                    break;
                case TIMEOUT:
                    logMsg("FIN_WAIT_2:  timeout", LOG_INFO, LOG_DEST_CONSOLE);
                    if (currentTCB->timeoutsCount)
                    {
                        TCP_Snd(currentTCB);
                    }
                    else
                    {
                        // Just reset the connection if there is no reply
                        currentTCB->flags = TCP_RST_FLAG;
                        if (TCP_Snd(currentTCB) == (TX_QUEUED || SUCCESS))
                        {
                            nextState = CLOSED;
                            TCB_Reset(currentTCB);
                        }
                    }
                    break;
                default:
                    break;
            }
            break;
        case CLOSE_WAIT:
            // This state is defined in RFC, but is not used in the application
            break;
        case CLOSING:
            switch (event)
            {
                case RCV_ACK:
                    logMsg("CLOSING: rx_ack", LOG_INFO, LOG_DEST_CONSOLE);
                    nextState = TIME_WAIT;
                    break;
                default:
                    break;
            }
            break;
        case LAST_ACK:
            // Check if the packet belongs to the curent TCB
            switch (event)
            {
                case RCV_FINACK:
                case RCV_ACK:
                    if ((currentTCB->destIP == receivedRemoteAddress) &&
                        (currentTCB->destPort == tcpHeader.sourcePort))
                    {
                        logMsg("LAST_ACK: rx_ack", LOG_INFO, LOG_DEST_CONSOLE);
                        nextState = CLOSED;
                        TCB_Reset(currentTCB);
                    }
                    break;
                case TIMEOUT:
                    if (currentTCB->timeoutsCount)
                    {
                        TCP_Snd(currentTCB);
                    }
                    else
                    {
                        // Just reset the connection if there is no reply
                        currentTCB->flags = TCP_RST_FLAG;
                        if (TCP_Snd(currentTCB) == (TX_QUEUED || SUCCESS))
                        {
                            nextState = CLOSED;
                            TCB_Reset(currentTCB);
                        }
                    }
                default:
                    break;
            }
            break;
        case TIME_WAIT:
            logMsg("Time Wait", LOG_INFO, LOG_DEST_CONSOLE);
            nextState = CLOSED;
            TCB_Reset(currentTCB);
            break;
        case CLOSED:
            switch (event)
            {
                case ACTIVE_OPEN:
                    logMsg("CLOSED: active_open", LOG_INFO, LOG_DEST_CONSOLE);
                    // Create and send a SYN packet
                    currentTCB->timeout = TCP_START_TIMEOUT_VAL;
                    currentTCB->timeoutReloadValue = TCP_START_TIMEOUT_VAL;
                    currentTCB->timeoutsCount = TCP_MAX_SYN_RETRIES;
                    currentTCB->flags = TCP_SYN_FLAG;
                    TCP_Snd(currentTCB);
                    nextState = SYN_SENT;
                    ret = SUCCESS;
                    break;
                case PASIVE_OPEN:
                    logMsg("CLOSED: passive_open", LOG_INFO, LOG_DEST_CONSOLE);
                    currentTCB->destIP = 0;
                    currentTCB->destPort = 0;
                    nextState = LISTEN;
                    ret = SUCCESS;
                    break;
                default:
                    break;
                }
            break;
        default:
            break;
    }
    currentTCB->connectionEvent = NOP; // Handling the event...
    currentTCB->fsmState = nextState;
    return ret;
}

void TCP_Init(void)
{
    // logMsg("TCP_Init", LOG_INFO, LOG_DEST_CONSOLE);
    PRINTF("<%s>", __FUNCTION__);
    tcbList = NULL;
    tcbListSize = 0;
    nextAvailablePort = LOCAL_TCP_PORT_START_NUMBER;
    nextSequenceNumber = 0;
}

error_msg TCP_SocketInit(tcpTCB_t *tcbPtr)
{
    // logMsg("TCP_SocketInit", LOG_INFO, LOG_DEST_CONSOLE);
    PRINTF("<%s", __FUNCTION__);
    error_msg ret = ERROR;

    // Verify that this socket is not in the list
    if (TCB_Check(tcbPtr) == ERROR)
    {
        TCB_Reset(tcbPtr);

        tcbPtr->localWnd = 0; // Input the RX buffer size
        tcbPtr->mss = TCP_MAX_SEG_SIZE;
        tcbPtr->fsmState = CLOSED;
        tcbPtr->connectionEvent = NOP;
        tcbPtr->rxBufferStart = NULL;
        tcbPtr->rxBufState = NO_BUFF;
        tcbPtr->txBufferStart = NULL;
        tcbPtr->txBufferPtr = NULL;
        tcbPtr->bytesToSend = 0;
        tcbPtr->bytesSent = 0;
        tcbPtr->payloadSave = false;
        tcbPtr->txBufState = NO_BUFF;
        tcbPtr->socketState = SOCKET_CLOSED;

        TCB_Insert(tcbPtr);
        ret = SUCCESS;
    }
    PRINTF("%s>", __FUNCTION__);
    return ret;
}

error_msg TCP_SocketRemove(tcpTCB_t *tcbPtr)
{
    PRINTF("<%s", __FUNCTION__);;
    error_msg ret = ERROR;

    // Verify that this socket is in the Closed State
    if (TCP_SocketPoll(tcbPtr) == SOCKET_CLOSING)
    {
        TCB_Remove(tcbPtr);
        ret = SUCCESS;
    }
    PRINTF("%s>", __FUNCTION__);;
    return ret;
}

socketState_t TCP_SocketPoll(tcpTCB_t *socket_ptr)
{
    PRINTF("<%s", __FUNCTION__);;
    socketState_t tmpSocketState = NOT_A_SOCKET;

    if (TCB_Check(socket_ptr) == SUCCESS)
    {
        tmpSocketState = socket_ptr->socketState;
    }

    PRINTF("%s>", __FUNCTION__);;
    return tmpSocketState;
}

error_msg TCP_Bind(tcpTCB_t *tcbPtr, uint16_t port)
{
    PRINTF("<%s", __FUNCTION__);;
    error_msg ret = ERROR;

    if (TCB_Check(tcbPtr) == SUCCESS)
    {
        tcbPtr->localPort = port;
        ret = SUCCESS;
    }
    PRINTF("%s>", __FUNCTION__);;
    return ret;
}

error_msg TCP_Listen(tcpTCB_t *tcbPtr)
{
    PRINTF("<%s", __FUNCTION__);;
    error_msg ret = ERROR;

    if (TCB_Check(tcbPtr) == SUCCESS)
    {
        tcbPtr->connectionEvent = PASIVE_OPEN;
        tcbPtr->socketState = SOCKET_IN_PROGRESS;
        tcbPtr->localSeqno = nextSequenceNumber;
        currentTCB = tcbPtr;
        if (tcbPtr->localPort == 0)
        {
            tcbPtr->localPort = nextAvailablePort++;
        }
        ret = TCP_FiniteStateMachine();
    }
    PRINTF("%s>", __FUNCTION__);;
    return ret;
}

error_msg TCP_Connect(tcpTCB_t *tcbPtr, sockaddr_in4_t *srvaddr)
{
    PRINTF("<%s", __FUNCTION__);;
    error_msg ret = ERROR;

    if (TCP_SocketPoll(tcbPtr) == SOCKET_CLOSED)
    {
        tcbPtr->destIP = srvaddr->addr.s_addr;
        tcbPtr->destPort = srvaddr->port;
        if (tcbPtr->localPort == 0)
        {
            // Using a random port for the local one
            tcbPtr->localPort = nextAvailablePort++;
        }

        tcbPtr->fsmState = CLOSED;
        tcbPtr->socketState = SOCKET_IN_PROGRESS;
        tcbPtr->localSeqno = nextSequenceNumber;
        tcbPtr->connectionEvent = ACTIVE_OPEN;

        currentTCB = tcbPtr;
        ret = TCP_FiniteStateMachine();
    }

    PRINTF("%s>", __FUNCTION__);;
    return ret;
}

error_msg TCP_Close(tcpTCB_t *tcbPtr)
{
    error_msg ret = ERROR;

    logMsg("tcp_close", LOG_INFO, LOG_DEST_CONSOLE);

    if (TCB_Check(tcbPtr) == SUCCESS)
    {
        tcbPtr->connectionEvent = CLOSE;

        tcbPtr->txBufState = NO_BUFF;
        tcbPtr->rxBufState = NO_BUFF;
        tcbPtr->txBufferPtr = NULL;
        tcbPtr->txBufferStart = NULL;
        tcbPtr->rxBufferPtr = NULL;
        tcbPtr->rxBufferStart = NULL;
        tcbPtr->bytesToSend = 0;
        tcbPtr->bytesSent = 0;
        tcbPtr->payloadSave = false;

        currentTCB = tcbPtr;
        ret = TCP_FiniteStateMachine();
    }
    return ret;
}

error_msg TCP_Send(tcpTCB_t *tcbPtr, uint8_t *data, uint16_t dataLen)
{
    error_msg ret = ERROR;

    if (TCP_SocketPoll(tcbPtr) == SOCKET_CONNECTED)
    {
        if (tcbPtr->txBufState == NO_BUFF)
        {
            if (data != NULL)
            {
                tcbPtr->txBufferStart = data;
                tcbPtr->txBufferPtr = tcbPtr->txBufferStart;
                tcbPtr->bytesToSend = dataLen;
                tcbPtr->txBufState = TX_BUFF_IN_USE;
                tcbPtr->bytesSent = dataLen;

                tcbPtr->timeout = TCP_START_TIMEOUT_VAL;
                tcbPtr->timeoutReloadValue = TCP_START_TIMEOUT_VAL;
                tcbPtr->timeoutsCount = TCP_MAX_RETRIES;

                tcbPtr->flags = TCP_ACK_FLAG;

                TCP_Snd(tcbPtr);
                ret = SUCCESS;
            }
        }
    }
    return ret;
}

error_msg TCP_SendDone(tcpTCB_t *tcbPtr)
{
    error_msg ret = ERROR;

    if (TCB_Check(tcbPtr) == SUCCESS)
    {
        if (tcbPtr->txBufState == NO_BUFF)
        {
            ret = SUCCESS;
        }
    }
    return ret;
}

error_msg TCP_InsertRxBuffer(tcpTCB_t *tcbPtr, uint8_t *data, uint16_t data_len)
{
    PRINTF("<%s", __FUNCTION__);;
    error_msg ret = ERROR;

    if (TCB_Check(tcbPtr) == SUCCESS)
    {
        if (tcbPtr->rxBufState == NO_BUFF)
        {
            if (data != NULL)
            {
                tcbPtr->rxBufferStart = data;
                tcbPtr->rxBufferPtr = tcbPtr->rxBufferStart;
                tcbPtr->localWnd = data_len; // Update the available receive windows
                tcbPtr->rxBufState = RX_BUFF_IN_USE;
                ret = SUCCESS;
            }
        }
    }
    PRINTF("%s>", __FUNCTION__);;
    return ret;
}

int16_t TCP_GetReceivedData(tcpTCB_t *tcbPtr)
{
    PRINTF("<%s", __FUNCTION__);;
    int16_t ret = 0;

    if (TCB_Check(tcbPtr) == SUCCESS)
    {
        if (tcbPtr->rxBufState == RX_BUFF_IN_USE)
        {
            ret = tcbPtr->rxBufferPtr - tcbPtr->rxBufferStart;

            if (ret != 0)
            {
                tcbPtr->localWnd = 0;
                tcbPtr->rxBufState = NO_BUFF;
            }
        }
    }
    PRINTF("%s>", __FUNCTION__);;
    return ret;
}

int16_t TCP_GetRxLength(tcpTCB_t *tcbPtr)
{
    int16_t ret = 0;

    if (TCB_Check(tcbPtr) == SUCCESS)
    {
        if (tcbPtr->rxBufState == RX_BUFF_IN_USE)
        {
            ret = tcbPtr->rxBufferPtr - tcbPtr->rxBufferStart;
        }
    }
    return ret;
}

void TCP_Update(void)
{
    tcpTCB_t *tcbPtr;
    tcbPtr = NULL;
    int count = 0;

    // Update sequence number and local port number in order to be different for each new connection
    nextSequenceNumber++;

    // Keep local port number in the general port range
    nextAvailablePort = nextAvailablePort + 1;
    if (nextAvailablePort < LOCAL_TCP_PORT_START_NUMBER)
    {
        nextAvailablePort = LOCAL_TCP_PORT_START_NUMBER;
    }

    tcbPtr = tcbList;
    while ((tcbPtr != NULL) && (count < tcbListSize))
    {
        if (tcbPtr->timeout > 0)
        {
            logMsg("tcp timeout", LOG_INFO, LOG_DEST_CONSOLE);
            tcbPtr->timeout = tcbPtr->timeout - 1;

            if (tcbPtr->timeout == 0)
            {
                // Making sure not to overwrite anything else
                if (tcbPtr->connectionEvent == NOP)
                {
                    int retries = TCP_MAX_RETRIES - tcbPtr->timeoutsCount;
                    if (retries < 0)
                    {
                        retries = 0;
                    }
                    tcbPtr->timeout = tcbPtr->timeoutReloadValue << retries;
                    // If not zero
                    if (tcbPtr->timeoutsCount != 0)
                        tcbPtr->timeoutsCount = tcbPtr->timeoutsCount - 1u;
                    tcbPtr->connectionEvent = TIMEOUT;
                    currentTCB = tcbPtr;
                    TCP_FiniteStateMachine();
                }
            }
        }
        tcbPtr = tcbPtr->nextTCB;
        count++;
    }
}

static error_msg TCP_TimoutRetransmit(void)
{
    currentTCB->txBufferPtr -= tcpDataLength;
    txBufferPtrForRetransmit = currentTCB->txBufferPtr;
    bytesToSendForRetransmit = tcpDataLength;
    currentTCB->localSeqno = localSeqnoForRetransmit;
    lastAckNumber = tcpHeader.ackNumber;
    return TCP_Snd(currentTCB);
}
