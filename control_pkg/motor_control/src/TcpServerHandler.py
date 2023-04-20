#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# @Author: wilson_t
# @Date:   2020-07-30 14:33:50
# @Last Modified by:   wilson_t
# @Last Modified time: 2020-08-03 23:13:17

import socket
from threading import Thread
import time
import numpy as np

class TcpServerHandler(Thread):
    def __init__(self, ip, port, proto):
        super(TcpServerHandler, self).__init__()
        self.socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        self.connection = None
        self.host = ip
        self.port = port
        self.isConnected = False
        self.isImageReadyToSend = False
        self.recvBuff = []
        self.RECV_BUFFER_SIZE = 10
        self.protocol = proto
        self.offlineCnt = 0
        # self.subThread = Thread(target = self.checkConnection)

    def checkConnection(self):
        try:
            self.connection.send("Y")
            self.isConnected = True
            return True
        except:
            print("Connection is lost")
            self.isConnected = False
            return False

    def setRecvBuffSize(self, size):
        self.RECV_BUFFER_SIZE = size

    def setProtocol(self, proto):
        self.protocol = proto

    def waitConnect(self):
        try:
            self.close()
            self.socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)  
            self.socket.bind((self.host, self.port))      
            self.socket.listen(5)
            print("wait connecting...")
            self.connection, addr = self.socket.accept()
            print('Connected by ', addr)
            self.isConnected = True
            self.isImageReadyToSend = False
        except:
            print("socket wrong!")

    def close(self):
        try:
            if(self.connection != None):
                try:
                    self.connection.close()
                    self.connection = None
                except:
                    pass
            if(self.socket != None):
                try:
                    self.socket.close()
                    self.socket = None
                except:
                    pass
        except:
            print("socket close failed!")

    def send(self, labelName):
        # print(type(labelName))
        try:
            if(type(labelName) == str):
                try:
                    data = labelName.encode("utf-8")
                except:
                    pass
                self.connection.send(data) #self.socketet Send
            elif(type(labelName) == bytes):
                self.connection.send(labelName)
            elif(type(labelName) == bytearray):
                self.connection.send(bytes(labelName))
            elif(type(labelName) == list):
                try:
                    #data=b''.join(map(lambda x:int.to_bytes(x,1,'little'),labelName))
                    print('origin data:{0}'.format(labelName))
                    data = bytes(labelName)
                    print(type(data))
                    print('transfer dat:{0}'.format(data))
                except:
                    print("list to bytes failed")
                    return False
                self.connection.send(data)
            else:
                try:
                    data = bytes(labelName)
                except:
                    print("to bytes failed")
                    return False
                self.connection.send(data)
            return True
        except:
            print("send failed")
            self.isConnected = False
            return False

    def recv(self):
        if(len(self.recvBuff) == 0):
            return None
        else:
            return self.recvBuff.pop(0)

    def calc_crc(self, data, len):
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

    def sendByteImage(self, bytesSeq, len):
        try:
            imageLen = len

            notify = [0 for _ in range(10)]
            notify[0] = 0xff
            notify[1] = 0xfe
            notify[2] = 0x10
            notify[3] = 4
            for i in range(4):
                notify[i+4] = (imageLen >> (24 - i*8)) & 0xff
            try:
                _crc = self.calc_crc(notify, 8)
                notify[8] = (_crc >> 8) & 0xff
                notify[9] = _crc & 0xff
                print(notify)
            except:
                print("CRC calc error")
                return False

            if(False == self.send(notify)):
                self.isConnected = False
                print("notify send failed")
                return False

            lastTime = time.time()
            # self.isImageReadyToSend = False
            while(self.isImageReadyToSend != True):
                if(time.time() - lastTime > 2.5):
                    break
            if(self.isImageReadyToSend):
                self.isImageReadyToSend = False
                if(False == self.send(bytesSeq)):
                    print("image send failed")
                    return False
            else:
                self.send("N")  # 超时，取消本次发送
                print("time out to receive apply")
                return False
        except:
            print("someting wrong!")
            return False
        return True

    def sendFileImage(self, filepath):
        try:
            file = open(filepath, 'rb')
            img = file.read()
            imageLen = len(img)
            return self.sendByteImage(img, imageLen)
        except:
            print("file not found")
            return False
    def sendMatImage(self, img):
        try:
            sendImg = img
            imageLen = len(sendImg)
            return self.sendByteImage(sendImg, imageLen)
        except:
            print("image transfer failed")
            return False

    def run(self):
        while(True):
            if(False == self.isConnected):
                self.waitConnect()
            if(self.isConnected):
                try:
                    msg = self.connection.recv(256)
                    if(msg == b'OK'):
                        self.isImageReadyToSend = True
                    elif(msg == b''):
                        self.offlineCnt += 1
                        if(self.offlineCnt >= 3):
                            print("connection lost!")
                            self.offlineCnt = 0
                            self.isConnected = False
                    else:
                        try:
                            msgDecode = self.protocol(msg)
                        except:
                            print("decode error")
                        if(msgDecode != None):
                            self.recvBuff.append(msgDecode)
                            if(len(self.recvBuff) > self.RECV_BUFFER_SIZE):
                                self.recvBuff.pop(0)
                        self.offlineCnt = 0
                except:
                    self.isConnected = False
                    print("recv failed")
            time.sleep(0.01)
