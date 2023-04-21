#include "robotsocket/robotsocket.h"
#include <stdio.h>
#include <string.h>

// 16进制char[]转char
// char dbuff[]={0x7B, 0x22, 0x78, 0x22, 0x3A, 0x22, 0x37, 0x22, 0x2C, 0x22, 0x79, 0x22, 0x3A, 0x22, 0x37, 0x22, 0x2C, 0x22, 0x69, 0x64, 0x22, 0x3A, 0x22, 0x31, 0x22, 0x7D};
// char sbuff[1024]="7B2278223A2237222C2279223A2237222C226964223A2231227D"

void RobotSocket::array2char(char *dbuff, char *sbuff)
{
  char highByte, lowByte;

  for (int i = 0; i < strlen(dbuff); i++)
  {
    highByte = dbuff[i] >> 4;
    lowByte = dbuff[i] & 0x0f;

    highByte += 0x30;

    if (highByte > 0x39)
      sbuff[i * 2] = highByte + 0x07;
    else
      sbuff[i * 2] = highByte;

    lowByte += 0x30;
    if (lowByte > 0x39)
      sbuff[i * 2 + 1] = lowByte + 0x07;
    else
      sbuff[i * 2 + 1] = lowByte;
  }
  sbuff[strlen(dbuff) * 2] = '\0';
  // printf("array2char: %s\n", sbuff);
}
