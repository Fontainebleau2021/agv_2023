#include <iostream>
#include <sstream>
#include <string>
#include <iomanip>

using namespace std;
 
string dec2hex(int i) //将int转成16进制字符串
{
	stringstream ioss; //定义字符串流
	string s_temp; //存放转化后字符
	// ioss << setiosflags(ios::uppercase) << hex << i; //以十六制(大写)形式输出
	ioss << resetiosflags(ios::uppercase) << hex << i; //以十六制(小写)形式输出//取消大写的设置
	ioss >> s_temp;
	return s_temp;
}
 
int main()
{
	int a = 2013;
	cout << "a = 0x" << dec2hex(a) << endl;
	
}