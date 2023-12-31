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
    
#include "ksz8851snl.h"
#include "memory_buffer.h"


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
#define TX_BUFFER_SIZE                  (TXEND - TXSTART) //6144

#define TX_BUFFER_MID                   ((TXSTART + TXEND) >> 1 ) //3072

#define SetBit( bitField, bitMask )     do{ bitField = bitField | bitMask; } while(0)
#define ClearBit( bitField, bitMask )   do{ bitField = bitField & (~bitMask); } while(0)
#define CheckBit( bitField, bitMask )   (bool)(bitField & bitMask)

volatile uint32 *csport;
uint16_t  cspinmask;

/*******************************************************************************/

/**
 * System Software Reset
 */
void KSZ88_SoftReset(void)
{
    KSZ88_RegWrite8(KSZ_GRR, GRR_GLOBAL_SOFT_RESET);
    delay(10);
    KSZ88_RegWrite8(KSZ_GRR, 0);
    delay(10);
}

//---------------------------------------------------------------------------------------------------------------------
void KSZ88_Init(void)
{
    PRINTF("<%s", __FUNCTION__);
    SPI.beginTransaction(SPISettings(12000000)); // lower speed
    csport = portSetRegister(ETH_CS_PIN);
    cspinmask = digitalPinToBitMask(ETH_CS_PIN);
    pinMode(ETH_CS_PIN, OUTPUT);
    pinMode(ETH_IRQ_PIN, INPUT_PULLUP);

    uint8_t cider1_ok = 0;
    uint8_t cider2_ok = 0;
    // check SPI
    while(1)
    {
        if (!cider2_ok) {
            uint16_t cider = KSZ88_RegRead8(KSZ_CIDER) + (KSZ88_RegRead8((ksz8851_registers_t)(KSZ_CIDER+1)) << 8); 
            if (cider != KSZ_CIDER_VALUE) {
                PRINTF("EROR: Wrong chip ID (8 bit read): %04X", cider);
                // while(1);
                delay(1000);
            } else
                cider2_ok = 1;
        }
        if (!cider1_ok) {
            uint16_t cider = KSZ88_RegRead16(KSZ_CIDER);
            if (cider != KSZ_CIDER_VALUE) {
                PRINTF("EROR: Wrong chip ID: %04X", cider);
                while(1);
            }
            cider1_ok = 1;
        }
        if (cider1_ok && cider2_ok) break;
    }
    while(1)
    {
        uint16_t ccr = KSZ88_RegRead16(KSZ_CCR);
        if (Serial.available()) //(ccr)
        {
            if (Serial.available()) Serial.read();
            if (ccr&CCR_EEPROM_PRESENCE) PRINTF("KSZ9951 EEPROM present ");
            if (ccr&CCR_SPI_BUS_MODE) PRINTF("KSZ9951 SPIbus mode active.");
            if (ccr&CCR_32PIN_CHIP_PACKAGE) PRINTF("KSZ9951 32 pin package.");
            break;
        }
        PRINTF("SPI communication error: check your hardware!");
        delay(1000);
    }

    // Software reset
    KSZ88_SoftReset();

    // enable energy detect mode (power save)
    KSZ88_BitFieldSet(KSZ_PMECR, PMECR_WAKEUP_TO_NORMAl);

    KSZ88_RegWrite16(KSZ_TXFDPR, TXFDPR_TXFPAI);
    KSZ88_RegWrite16(KSZ_TXCR, TXCR_TXFCE | TXCR_TXPE | TXCR_TXCE);

    KSZ88_RegWrite16(KSZ_RXFDPR, RXFDPR_RXFPAI);
    KSZ88_RegWrite16(KSZ_RXCR1, RXCR1_RXUDPFCC | RXCR1_RXTCPFCC | RXCR1_RXIPFCC | RXCR1_RXPAFMA | RXCR1_RXFCE |
                     RXCR1_RXMAFMA | RXCR1_RXME | RXCR1_RXUE);
    KSZ88_RegWrite16(KSZ_RXCR2, RXCR2_DATA_BURST_FRAME | RXCR2_IUFFP | RXCR2_RXIUFCEZ | RXCR2_UDPLFE | RXCR2_RXICMPFCC);
    KSZ88_RegWrite16(KSZ_RXQCR, RXQCR_RXIPHTOE | RXQCR_RXFCTE | RXQCR_ADRFE);

    KSZ88_RegWrite16(KSZ_RXDTTR, RXDTTR_VALUE);
    KSZ88_RegWrite16(KSZ_RXDBCTR, RXDBCTR_VALUE);

    KSZ88_RegWrite8(KSZ_RXFCT, RXFCT_VALUE);
    KSZ88_RegWrite16(KSZ_FCLWR, FCLWR_VALUE);
    KSZ88_RegWrite16(KSZ_FCHWR, FCHWR_VALUE);
    KSZ88_RegWrite16(KSZ_ISR, ISR_CLEAR_ALL);

    // KSZ88_RegWrite16(KSZ_IER, IER_LCIE | IER_TXIE | IER_RXIE | IER_RXOIE | IER_LDIE);
    KSZ88_BitFieldSet(KSZ_TXCR, TXCR_TXE); // transmit enable
    KSZ88_BitFieldSet(KSZ_RXCR1, RXCR1_RXE); // receive enable
    PRINTF("%s>", __FUNCTION__);;
}

/**
 * Read 1 byte from KSZ Register
 * @param address
 * @return
 */
uint8_t KSZ88_RegRead8(ksz8851_registers_t address)
{
    uint16_t cmd = (read_io | (1 << ((address & LSBMask) + 10)) | (address << 2));

    ETH_NCS_LOW();
    ETH_SPI_WRITE16(cmd);
    uint8_t value = ETH_SPI_READ8();
    ETH_NCS_HIGH();

    PRINTF("<%s addr: %02X, cmd: %04X, ret: %04X", __FUNCTION__, address, cmd, value);
    return value;   
}

/**
 * Read 2 byte from KSZ Register
 * @param address
 * @return
 */

uint16_t KSZ88_RegRead16(ksz8851_registers_t address)
{
    //Checking to find if the register address is even or odd to use the correct mask value
    uint16_t cmd = (read_io | (wordEnable << ((address & 0x02) + 10)) | (address << 2));

    ETH_NCS_LOW();
    ETH_SPI_WRITE16(cmd);
    // ((char *) &value)[0] = ETH_SPI_READ8();
    // ((char *) &value)[1] = ETH_SPI_READ8();
    uint16_t value = byteSwap16(ETH_SPI_READ16());
    ETH_NCS_HIGH();
    PRINTF("<%s addr: %02X, ret: %04X", __FUNCTION__, address, value);
    return (value);
}
#if 0 // not used
/**
 * Read 4 byte from KSZ Register
 * @param address
 * @return
 */
uint32_t KSZ88_RegRead32(ksz8851_registers_t address)
{
    uint32_t value;
    uint16_t cmd = ((read_io << 8)| (dwordEnable << 10)| (address << 2));
    
    ETH_NCS_LOW();
    ETH_SPI_WRITE16(cmd);
    // ((char *) &value)[0] = ETH_SPI_READ8();
    // ((char *) &value)[1] = ETH_SPI_READ8();
    // ((char *) &value)[2] = ETH_SPI_READ8();
    // ((char *) &value)[3] = ETH_SPI_READ8();
    ETH_SPI_READ_BLOCK((uint8_t*)&value, 4);
    ETH_NCS_HIGH();
    
    return value;
}
#endif
/**
 * Write 1 byte to KSZ Register
 * @param address
 * @param value
 */
void KSZ88_RegWrite8(ksz8851_registers_t address, uint8_t value)
{
    PRINTF("<%s addr: %02X, val: %04X", __FUNCTION__, address, value);
    uint16_t cmd = (write_io | ((1 << (address & LSBMask)) << 10) | (address << 2));

    ETH_NCS_LOW();
    ETH_SPI_WRITE16(cmd);
    ETH_SPI_WRITE8(value);
    ETH_NCS_HIGH();
    if (0) {
        // read back and compare value
        uint8_t rd = KSZ88_RegRead8(address);
        if (address!=KSZ_GRR && address!=KSZ_RXFCT)
            if (rd != value) while(1);
    }
}

/**
 * Write 2 byte to KSZ Register
 * @param address
 * @param value
 */
void KSZ88_RegWrite16(ksz8851_registers_t address, uint16_t value)
{
    PRINTF("<%s addr: %02X, val: %04X", __FUNCTION__, address, value);
    uint16_t cmd = (write_io | (wordEnable << ((address & 0x02) + 10)) | (address << 2));
    
    ETH_NCS_LOW();
    ETH_SPI_WRITE16(cmd);
    // ETH_SPI_WRITE8(((char *) &value)[0]);
    // ETH_SPI_WRITE8(((char *) &value)[1]);
    ETH_SPI_WRITE16(byteSwap16(value));
    ETH_NCS_HIGH();
    if (0) {
      // read back and compare value
        uint16_t rd = KSZ88_RegRead16(address);
        if (address!=KSZ_RXCR2 && address!=KSZ_ISR && address!=KSZ_PMECR)
            if (rd != value) while(1);
    }
}
#if 0 // not used
/**
 * Write 4 byte to KSZ Register
 * @param address
 * @param value
 */
void KSZ88_RegWrite32(ksz8851_registers_t address, uint32_t value)
{
    uint16_t cmd = ((write_io << 8)| (dwordEnable << 10)| (address << 2));
    
    ETH_NCS_LOW();
    ETH_SPI_WRITE16(cmd);
    // ETH_SPI_WRITE8(((char *) &value)[0]);
    // ETH_SPI_WRITE8(((char *) &value)[1]);
    // ETH_SPI_WRITE8(((char *) &value)[2]);
    // ETH_SPI_WRITE8(((char *) &value)[3]);
    ETH_SPI_WRITE_BLOCK(&value, 4);
    ETH_NCS_HIGH(); 
}
#endif
/**
 * Bit Field Clear
 * @param address
 * @param value
 */
void KSZ88_BitFieldClear(ksz8851_registers_t address, uint16_t value) 
{
    uint16_t temp = KSZ88_RegRead16(address);
    temp = (temp & (~value));
    KSZ88_RegWrite16(address, temp);
}

/**
 * Bit Field Set
 * @param address
 * @param value
 */
void KSZ88_BitFieldSet(ksz8851_registers_t address, uint16_t value)
{
    uint16_t temp = KSZ88_RegRead16(address);
    temp = temp | value;
    KSZ88_RegWrite16(address, temp);
}

/**
 * Returns the available space size in the Ethernet TX Buffer
 * @param 
 * @return available space left in the TX buffer
 */
uint16_t KSZ88_GetFreeTxBufferSize(void)
{
    uint16_t freespace = KSZ88_RegRead16(KSZ_TXFDPR);
    freespace &= TX_BUFFER_FREE; 
    return (uint16_t)((uint16_t)TXEND - freespace);
}
