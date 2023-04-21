#include"robotsocket/robotsocket.h"
#include<string.h>
#include<iostream>
#include<unistd.h>

using namespace std;

int hexCharToInt(char c)  
{   
        if (c >= '0' && c <= '9') return (c - '0');  
        if (c >= 'A' && c <= 'F') return (c - 'A' + 10);  
        if (c >= 'a' && c <= 'f') return (c - 'a' + 10);  
        return 0;  
}  
  
char* hexstringToBytes(string s)  
{           
        int sz = s.length();  
        char *ret = new char[sz/2];  
        for (int i=0 ; i <sz ; i+=2) {  
            ret[i/2] = (char) ((hexCharToInt(s.at(i)) << 4)  
                                | hexCharToInt(s.at(i+1)));  
        }  
        return ret;  
}  
  
string bytestohexstring(char* bytes,int bytelength)  
{  
  string str("");  
  string str2("0123456789abcdef");   
   for (int i=0;i<bytelength;i++) {  
     int b;  
     b = 0x0f&(bytes[i]>>4);  
     char s1 = str2.at(b);  
     str.append(1,str2.at(b));            
     b = 0x0f & bytes[i];  
     str.append(1,str2.at(b));  
     char s2 = str2.at(b);  
   }  
   return str;  
}  

//ascii转16进制
//char ascii[]="{"x":"7","y":"7","id":"1"}"
//char hex[] = "7b2278223a2237222c2279223a2237222c226964223a2231227d"

void RobotSocket::ascii2hex(char* ascii, char* hex)  
{
        std::string result;
        result = bytestohexstring(ascii,strlen(ascii));  

        for(int i=0;i<result.length();i++){
                hex[i] = result[i];
                hex[i+1] = '\0';
        }
        
        // printf("ascii2hex: %s\n",hex);

}