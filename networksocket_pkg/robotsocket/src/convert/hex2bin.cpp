#include <iostream>
#include "robotsocket/harvestingRobotClient.h"

void HarvestingRobotClient::hex2bin(unsigned char in, char *bin_str, int str_size)
{
    int hex = in;
    for (int i = 0; i !=str_size; ++i)
    {
        bin_str[str_size - 1 - i] = hex % 2 + '0';
        hex /= 2;
    }
}