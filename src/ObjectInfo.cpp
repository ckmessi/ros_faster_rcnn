#include "ObjectInfo.h"


vector<ObjectInfo> parseObjectInfoList(string str, char c)
{
	// list存的每个字符串是一个对象信息
	vector<string> list;
    int i = 0;
    int lastPos = 0;
	while(i < (int)str.length()){
		if(str[i] == c){
			string temp = str.substr(lastPos, i-lastPos);
			list.push_back(temp);
			lastPos = i+1;			
		}
		i++;
    }
    string temp = str.substr(lastPos);
    list.push_back(temp);
    
    // objectList存的每个对象是一个对象
    vector<ObjectInfo> objectList;
    i = 0;
    while(i < (int)list.size() - 4){
		int x1 = atoi(list[i].c_str());
		i++;
		int y1 = atoi(list[i].c_str());
		i++;
		int x2 = atoi(list[i].c_str()); 
		i++;
		int y2 = atoi(list[i].c_str());
		i++;
		string title = list[i];
		i++;
		ObjectInfo oi(x1, y1, x2, y2, title);
		objectList.push_back(oi);
    } 
    return objectList;	
}
