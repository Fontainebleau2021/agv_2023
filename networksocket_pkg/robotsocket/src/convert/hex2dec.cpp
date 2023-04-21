#include <iostream>
#include <string>
#include <algorithm>
#include <sstream>
#include "robotsocket/harvestingRobotClient.h"

using namespace std;

// 16进制char转16进制string
string hexchar2hexstr(unsigned char* c){
      char h=*c>>4;
      if(h>=0&&h<=9){
          h=h+0x30;
      }else if(h>9&&h<=15){
          h=h+'a'-10;
      }else{return "";}
  
      char l=*c&0x0f;
  
      if(l>=0&&l<=9){
          l=l+0x30;
      }else if(l>9&&l<=15){
          l=l+'a'-10;
      }else{return "";}
      string s;
      s.append(&h,1);
      s.append(&l,1);
      return s;
  }

//十六进制string转十进制int，length为字节数，-表示无符号整型
int HexStrToDecInt(string str ,int length)
{   
    if(length == -1){
        uint8_t iDec=0;
        sscanf(str.c_str(), "%hhx", &iDec);
        return iDec;
    }else if(length == 1){
        int8_t iDec=0;
        sscanf(str.c_str(), "%hhx", &iDec);
        return iDec;
    }else if(length == -2){
        uint16_t iDec=0;
        sscanf(str.c_str(), "%hx", &iDec);
        return iDec;
    }else if(length == 2){
        int16_t iDec=0;
        sscanf(str.c_str(), "%hx", &iDec);
        return iDec;
    }else if(length == -3){
        uint32_t iDec=0;
        sscanf(str.c_str(), "%x", &iDec);
        return iDec;
    }else if(length == 3){
        int32_t iDec=0;
        sscanf(str.c_str(), "%x", &iDec);
        return iDec;
    }
	return 0;
}

int HarvestingRobotClient::hexChar2decInt(unsigned char* hex_char, int length){
    string str="";
    if(length ==1||length==-1){
        str = hexchar2hexstr(hex_char);
    }else if(length ==2||length==-2){
        str=hexchar2hexstr(hex_char)+hexchar2hexstr(hex_char+1);
    }else if(length ==3||length==-3){
        str=hexchar2hexstr(hex_char)+hexchar2hexstr(hex_char+1)+hexchar2hexstr(hex_char+2);
    }
    int dec_int = HexStrToDecInt(str,length);
    // cout<<"str: "<<str<<endl;
    // cout << "十进制：" << dec_int << endl;
    return dec_int;
}
