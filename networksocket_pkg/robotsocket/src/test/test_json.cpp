#include<string>
#include<iostream>
#include<fstream>
#include <jsoncpp/json/json.h> 
using namespace std;
 
int main()
{
Json::Value root;
Json::Reader reader;
Json::FastWriter fwriter;
Json::StyledWriter swriter;
Json::Value array;
 
std::ifstream ifs("src/robotsocket/src/test/waypoint.json");//open file example.json

if(!reader.parse(ifs, root)){
   // fail to parse
}
else{
   // success
   array = root["array"];
   std::cout<<array.size()<<endl;
	for (int index = 0; index < array.size(); ++index) {   // Iterates over the sequence elements.
        //cout << "Element " << index << " in array: " << array[index].asString() << endl;
	Json::Value position = array[index];
	cout << "Element: " << index << " x: " << position["x"].asFloat() <<"  y:"<< position["y"].asFloat() <<"  z:"<< position["z"].asFloat()<<"  w:"<< position["w"].asFloat()<< endl;
	position["x"]=Json::Value(1234);
	array[index] = position;
    }
}
 
root["array"] = array;
std::string str = fwriter.write(array);
std::ofstream ofs("src/robotsocket/src/test/example_fast_writer.json");
ofs << str;
ofs.close();
 
str = swriter.write(root);
ofs.open("src/robotsocket/src/test/example_styled_writer.json");
ofs << str;
ofs.close();
return 0;
}