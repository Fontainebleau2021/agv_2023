# -*- coding: utf-8 -*-
# @Author: wilson_t
# @Date:   2020-07-30 21:38:39
# @Last Modified by:   wilson_t
# @Last Modified time: 2020-08-02 18:05:42

import numpy as np

STOP = 0x00
FORWARD = 0x01
BACKWARD = 0x02
TURN_LEFT = 0x03
TURN_RIGHT = 0x04
ROCKER_CTRL = 0x05
TARGET_POS_DIR_CTRL = 0x06
TARGET_POS_CTRL = 0x07
TRACE_PHOTO_UPLOAD = 0x08
BUILD_GRAPH_UPLOAD = 0x09
MULTI_TARGET_POS = 0x0A
MULTI_TARGET_POS_DIR = 0x0B
STM32_CHARGE = 0x12


def calc_crc(data, len):
    crc = 0xFFFF
    for i in range(len):
        pos = data[i]
        crc ^= pos
        for i in range(8):
            if ((crc & 1) != 0):
                crc >>= 1
                crc ^= 0xA001
            else:
                crc >>= 1
    return crc

def customProtocol(msg):
    data = np.array(bytearray(msg))
    length = len(data)
    if(length < 4 or data[0] != 0xff or data[1] != 0xfe):
        return None
    funcCode = data[2]
    dataLen = data[3]
    if(funcCode == STOP):
        return [0]
    calcCrc = calc_crc(data, length - 2)
    recvCrc = (data[-2] << 8) | data[-1]
    if(recvCrc != calcCrc):
        return None

    if(funcCode == FORWARD or funcCode == BACKWARD or funcCode == TURN_LEFT or funcCode == TURN_RIGHT):
        if(length != 8):
            return None
        return [funcCode, (data[4] << 8) | data[5]]
    elif(funcCode==STM32_CHARGE):
        return [funcCode, data[4],data[5],data[6]]
    elif(funcCode == ROCKER_CTRL):
        if(length != 10):
            return None
        return [funcCode, (data[4] << 8) | data[5], (data[6] << 8) | data[7]]
    elif(funcCode == TARGET_POS_CTRL):
        if(length != 10):
            return None
        return [funcCode, (data[4] << 8) | data[5], (data[6] << 8) | data[7]]
    elif(funcCode == TARGET_POS_DIR_CTRL):
        if(length != 12):
            return None
        return [funcCode, (data[4] << 8) | data[5], (data[6] << 8) | data[7], (data[8] << 8) | data[9]]
    elif(funcCode == TRACE_PHOTO_UPLOAD or funcCode == BUILD_GRAPH_UPLOAD):
        if(length != 7):
            return None
        return [funcCode, data[4]]
    elif(funcCode == MULTI_TARGET_POS):
        posNums = data[3]
        if((length - 6) != (posNums * 4)):
            print("length not right")
            return None
        res = [funcCode]
        for i in range(posNums):
            x = (data[i*4+4] << 8) | data[i*4+5]
            y = (data[i*4+6] << 8) | data[i*4+7]
            res.append(x)
            res.append(y)
        return res
    elif(funcCode == MULTI_TARGET_POS_DIR):
        print("get multi target and dir")
        posNums = data[3]
        if(length - 8 != posNums*4):
            print("length not right")
            return None
        res = [funcCode]
        for i in range(posNums):
            x = (data[i*4+4] << 8) | data[i*4+5]
            y = (data[i*4+6] << 8) | data[i*4+7]
            res.append(x)
            res.append(y)
        dir = (data[posNums*4+4] << 8) | data[posNums*4+5]
        res.append(dir)
        return res
    

    return None