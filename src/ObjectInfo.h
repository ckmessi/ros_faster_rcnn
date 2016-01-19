#include <iostream>
#include <vector>
#include <string>
#include <cstdlib>

using namespace std;


class ObjectInfo{
    
public:
    int x1;
    int y1;
    int x2;
    int y2;

    string category;
	
public: 
    ObjectInfo(int x1, int y1, int x2, int y2, string category){
		this->x1 = x1;
		this->y1 = y1;
		this->x2 = x2;
		this->y2 = y2;
		this->category = category;
    }

   
    // 根据

};

// 根据字符串解析出检测目录
vector<ObjectInfo> parseObjectInfoList(string objectInfoStr, char c);
