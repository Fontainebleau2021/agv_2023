#include "robotsocket/robotsocket.h"
#include <iostream>

using namespace std;

int RobotSocket::action2_start(char *recv2, char *recv2_send, int recv2_length)
{

    // 显示recv2的内容
    // cout << "recv2:";
    // display(recv2, recv2_length);

    // 删除前5位
    char recv2_cut[1024];
    memset(recv2_cut, 0, sizeof(recv2_cut));
    strncpy(recv2_cut, recv2 + 5, recv2_length - 5); // 从recv2+5开始到结尾的字符复制到recv2_cut中
    recv2_cut[recv2_length - 5] = '\0';

    // 显示recv2_cut的内容
    // cout << "recv2_cut:";
    // display(recv2_cut, strlen(recv2_cut));

    // 把char数组拼成一整个char
    char recv2_char[1024];
    memset(recv2_char, 0, sizeof(recv2_char));
    array2char(recv2_cut, recv2_char);

    // 16进制转ascii
    char recv2_ascii[1024];
    memset(recv2_ascii, 0, sizeof(recv2_ascii));
    hex2ascii(recv2_char, recv2_ascii);
    
    // 读取收到的json指令，生成回复内容
    char recv2_json[1024];
    memset(recv2_json, 0, sizeof(recv2_json));
    (this->recv2_json_start)(recv2_ascii, recv2_json);

    // ascii转16进制
    char recv2_hex[1024];
    memset(recv2_hex, 0, sizeof(recv2_hex));
    ascii2hex(recv2_json, recv2_hex);

    // 16进制char转char[]
    char recv2_array[1024];
    memset(recv2_array, 0, sizeof(recv2_array));
    char2array(recv2_hex, recv2_array);

    // 添加前7位
    for (int i = 0; i < 7; i++)
    {
        recv2_send[i] = cmd2_head[i];
    };
    for (int i = 0; i < strlen(recv2_array); i++)
    {
        recv2_send[i + 7] = recv2_array[i];
    };

    // cout << "cmd2_start:";
    // display(recv2_send, sizeof(cmd2_head) + strlen(recv2_array));

    return sizeof(cmd2_head) + strlen(recv2_array);
};

int RobotSocket::action2(char *recv2, char *recv2_send, int recv2_length)
{

    // 显示recv2的内容
    // cout << "recv2:";
    // display(recv2, recv2_length);

    // 删除前5位
    char recv2_cut[1024];
    memset(recv2_cut, 0, sizeof(recv2_cut));
    strncpy(recv2_cut, recv2 + 5, recv2_length - 5); // 从recv2+5开始到结尾的字符复制到recv2_cut中
    recv2_cut[recv2_length - 5] = '\0';

    // 显示recv2_cut的内容
    // cout << "recv2_cut:";
    // display(recv2_cut, strlen(recv2_cut));

    // 把char数组拼成一整个char
    char recv2_char[1024];
    memset(recv2_char, 0, sizeof(recv2_char));
    array2char(recv2_cut, recv2_char);

    // 16进制转ascii
    char recv2_ascii[1024];
    memset(recv2_ascii, 0, sizeof(recv2_ascii));
    hex2ascii(recv2_char, recv2_ascii);

    // 读取收到的json指令，生成回复内容
    char recv2_json[1024];
    memset(recv2_json, 0, sizeof(recv2_json));
    (this->recv2_json)(recv2_ascii, recv2_json);

    // ascii转16进制
    char recv2_hex[1024];
    memset(recv2_hex, 0, sizeof(recv2_hex));
    ascii2hex(recv2_json, recv2_hex);

    // 16进制char转char[]
    char recv2_array[1024];
    memset(recv2_array, 0, sizeof(recv2_array));
    char2array(recv2_hex, recv2_array);

    // 添加前7位
    for (int i = 0; i < 7; i++)
    {
        recv2_send[i] = cmd2_head[i];
    };
    for (int i = 0; i < strlen(recv2_array); i++)
    {
        recv2_send[i + 7] = recv2_array[i];
    };

    // cout << "cmd2:";
    // display(recv2_send, sizeof(cmd2_head) + strlen(recv2_array));

    return sizeof(cmd2_head) + strlen(recv2_array);
};

int RobotSocket::action2_over(char *recv2, char *recv2_send, int recv2_length)
{

    // 显示recv2的内容
    // cout << "recv2:";
    // display(recv2, recv2_length);

    // 删除前5位
    char recv2_cut[1024];
    memset(recv2_cut, 0, sizeof(recv2_cut));
    strncpy(recv2_cut, recv2 + 5, recv2_length - 5); // 从recv2+5开始到结尾的字符复制到recv2_cut中
    recv2_cut[recv2_length - 5] = '\0';

    // 显示recv2_cut的内容
    // cout << "recv2_cut:";
    // display(recv2_cut, strlen(recv2_cut));

    // 把char数组拼成一整个char
    char recv2_char[1024];
    memset(recv2_char, 0, sizeof(recv2_char));
    array2char(recv2_cut, recv2_char);

    // 16进制转ascii
    char recv2_ascii[1024];
    memset(recv2_ascii, 0, sizeof(recv2_ascii));
    hex2ascii(recv2_char, recv2_ascii);

    // 读取收到的json指令，生成回复内容
    char recv2_json[1024];
    memset(recv2_json, 0, sizeof(recv2_json));
    (this->recv2_json)(recv2_ascii, recv2_json);

    // ascii转16进制
    char recv2_hex[1024];
    memset(recv2_hex, 0, sizeof(recv2_hex));
    ascii2hex(recv2_json, recv2_hex);

    // 16进制char转char[]
    char recv2_array[1024];
    memset(recv2_array, 0, sizeof(recv2_array));
    char2array(recv2_hex, recv2_array);

    // 添加前7位
    for (int i = 0; i < 7; i++)
    {
        recv2_send[i] = cmd2_over[i];
    };
    for (int i = 0; i < strlen(recv2_array); i++)
    {
        recv2_send[i + 7] = recv2_array[i];
    };

    // cout << "cmd2_over:";
    // display(recv2_send, sizeof(cmd2_over) + strlen(recv2_array));

    return sizeof(cmd2_over) + strlen(recv2_array);
};

int RobotSocket::action3(char *recv3, char *recv3_send, int recv3_length)
{
    // 生成回复内容json
    char recv3_json[1024];
    memset(recv3_json, 0, sizeof(recv3_json));
    (this->recv3_json)(recv3_json);

    // ascii转16进制
    char recv3_hex[1024];
    memset(recv3_hex, 0, sizeof(recv3_hex));
    ascii2hex(recv3_json, recv3_hex);

    // 16进制char转char[]
    char recv3_array[1024];
    memset(recv3_array, 0, sizeof(recv3_array));
    char2array(recv3_hex, recv3_array);

    for (int i = 0; i < 6; i++)
    {
        recv3_send[i] = cmd3_head[i];
    };
    for (int i = 0; i < strlen(recv3_array); i++)
    {
        recv3_send[i + 6] = recv3_array[i];
    };

    // cout << "cmd3:";
    // display(recv3_send, sizeof(cmd3_head) + strlen(recv3_array));

    return sizeof(cmd3_head) + strlen(recv3_array);
};

int RobotSocket::action4(char *recv4, char *recv4_send, int recv4_length)
{
    // 生成回复内容json
    char recv4_json[1024];
    memset(recv4_json, 0, sizeof(recv4_json));
    (this->recv4_json)(recv4_json);

    // ascii转16进制
    char recv4_hex[1024];
    memset(recv4_hex, 0, sizeof(recv4_hex));
    ascii2hex(recv4_json, recv4_hex);

    // 16进制char转char[]
    char recv4_array[1024];
    memset(recv4_array, 0, sizeof(recv4_array));
    char2array(recv4_hex, recv4_array);

    for (int i = 0; i < 6; i++)
    {
        recv4_send[i] = cmd4_head[i];
    };
    for (int i = 0; i < strlen(recv4_array); i++)
    {
        recv4_send[i + 6] = recv4_array[i];
    };

    // cout << "cmd4-9:";
    // display(recv4_send, sizeof(cmd4_head) + strlen(recv4_array));

    return sizeof(cmd4_head) + strlen(recv4_array);
};