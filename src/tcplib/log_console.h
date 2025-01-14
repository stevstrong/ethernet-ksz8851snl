/**
 * Log Console Header File
 *
 * @file log_console.h
 * 
 * @ingroup tcpiplite
 * 
 * @brief  This file provides the API implementation for sending log messages to the console.
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

#ifndef LOG_CONSOLE_H
#define	LOG_CONSOLE_H

#include <stdint.h>

// Log Console Functions
/**
 @ingroup tcpiplite
 @brief Logs a message to console using UART.
 @param *message
 @param priorityVal Message priority
 @retval 1
 */
uint8_t logConsole(const char *message, uint8_t priorityVal);

#ifdef __cplusplus
extern "C" {
#endif
extern void PRINTF(const char *, ...);
#ifdef __cplusplus
}
#endif

#endif	/* LOG_CONSOLE_H */
