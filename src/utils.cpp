#include <stdio.h>
#include <stdarg.h>
#include <Arduino.h>

extern "C" {

char buf[4096];
char ip_buf[16];
char mac_buf[20];

void DBG_PRINTF(const char * format, ...)
{
    va_list ap;
    va_start(ap, format);
    vsprintf(buf, format, ap);
    va_end(ap);
    Serial.println(buf);
    // delay(10);
}

void PRINTF(const char * format, ...)
{
    va_list ap;
    va_start(ap, format);
    vsprintf(buf, format, ap);
    va_end(ap);
    Serial.println(buf);
    // delay(10);
}

char * printIP(uint8_t * address)
{
    sprintf(ip_buf,"%u.%u.%u.%u",address[0],address[1],address[2],address[3]);
    return ip_buf;
}

char * printMAC(uint8_t * address)
{
    sprintf(mac_buf,"%2x:%2x:%2x:%2x:%2x:%2x",address[0],address[1],address[2],address[3],address[4],address[5]);
    return mac_buf;
}


}
