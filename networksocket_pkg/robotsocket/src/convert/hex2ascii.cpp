#include"robotsocket/robotsocket.h"
#include <iostream>
#include<string.h>

using namespace std;
unsigned int ssstrlen(const char *str)
{
    const char *cp = str;
    while(*cp++);
    return (cp - str -1);
}

int hex2char(uint8_t c)
{
    return ((c >= '0') && (c <= '9')) ? int(c - '0') :
           ((c >= 'A') && (c <= 'F')) ? int(c - 'A' + 10) :
           ((c >= 'a') && (c <= 'f')) ? int(c - 'a' + 10) :
           -1;
}

//16进制转ascii
//char hex[] = "7B2278223A2237222C2279223A2237222C226964223A2231227D"
//char ascii[]="{"x":"7","y":"7","id":"1"}"

int RobotSocket::hex2ascii(char* hex, char* ascii)
{
    int hexLen = ssstrlen(hex);
    int asciiLen = 0;

    for (int i = 0, cnt = 0; i<hexLen; i++)
    {
        char c = hex2char(hex[i]);

        if (-1 == c)
            continue;
        if(cnt) {
            cnt = 0;
            ascii[asciiLen++] += c;
        } else {
            cnt = 1;
            ascii[asciiLen] = c << 4;
        }
    }
    ascii[asciiLen++] = 0;
    // printf("Hex2Ascii: %s\n",ascii);
    return asciiLen;
}
