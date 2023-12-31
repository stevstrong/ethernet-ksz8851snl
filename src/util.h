#ifndef UTIL_H
#define UTIL_H

#include <wirish.h>

#ifndef word
 #define word(a, b) ( (uint16_t)((a)<<8) | (b) )
#endif

#ifndef htons
 #define htons(x) __builtin_bswap16(x)
 #define ntohs(x) htons(x)
#endif

#ifndef htonl
 #define htonl(x) __builtin_bswap32(x)
 #define ntohl(x) htonl(x)
#endif


#ifdef __cplusplus
extern "C" {
#endif

void PRINTF(const char * format, ...); // should be declared in each module in part

char * printIP(uint8_t * address);
char * printMAC(uint8_t * address);

#ifdef __cplusplus
}
#endif

#endif
