#include"robotsocket/robotsocket.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
 
int char2bits(char ch)
{
    int bits = 0;
    if (ch >= 'a' && ch <= 'z') 
    {
        bits = ch - 'a' + 10;
    } 
    else if (ch >= 'A' && ch <= 'Z') 
    {
        bits = ch - 'A' + 10;
    } 
    else if (ch >= '0' && ch <= '9') 
    {
        bits = ch - '0';
    } 
    else
    {
        bits = -1;
    }
    return bits;
}
 
int RobotSocket::char2array(char *hex, char *bytes) 
{
    int len = strlen(hex);
    int nbytes = len/2;
 
    for (int n = 0; n < nbytes; ++ n) 
    {
        int lndx = n * 2;
        int rndx = lndx + 1;
        int lbits = char2bits(hex[lndx]);
        int rbits = char2bits(hex[rndx]);
        if (lbits == -1 || rbits == -1)
        {
            return -1;
        }
        bytes[n] = (lbits << 4) | rbits;
    }
 
    return nbytes;
}