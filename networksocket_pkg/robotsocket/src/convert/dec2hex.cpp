#include <iostream>
#include <sstream>
#include <string>
#include <iomanip>
#include <stdio.h>
#include <stdlib.h>
#include"robotsocket/harvestingRobotClient.h"

//i要转化的十进制整数，width转化后的宽度，位数不足则补0 
string dec2hex(int i, int width)
 {
    stringstream ioss; //定义字符串流 
    string s_temp; //存放转化后字符
    ioss << hex << i; //以十六制形式输出 
    ioss >> s_temp; 
    if (width > s_temp.size()) { 
        string s_0(width - s_temp.size(), '0'); //位数不够则补0                 
        s_temp = s_0 + s_temp; //合并
    } 
    string s = s_temp.substr(s_temp.length() - width,   s_temp.length()); //取右width位 
    return s;
 }    

// int转为16进制char[]，i为输入int型整数，length为转换后长度，不足补零，hex为输出char[];
// nbytes是字节数，length = nbytes*2
void HarvestingRobotClient::int2hex(int i, int length, char * hex)
{
	string s = dec2hex(i,length);
    char res[length];
    for(int i=0;i<length;i++){
        res[i]=s[i];
    }
    res[length]='\0';
    int nbytes = char2array(res,hex);
    cout<<"i: "<<i<<" length: "<<length<<"nbytes: "<<nbytes<<endl;
    if(i<0){
        cout<<"int2hex: ";
        for (int i = 0 ; i < nbytes; ++ i) 
        {
            printf("%hhx ", hex[i]);
        }
        cout<<endl;
    }else{
        cout<<"int2hex: ";
        for (int i = 0 ; i < nbytes; ++ i) 
        {
            printf("%02x ", hex[i]);
        }
        cout<<endl;
    }
}