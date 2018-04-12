/**************************************************************************************
 GPS 轨迹相关结构的定义
 **************************************************************************************/
#ifndef   GPS_DATA_STRUCT
#define  GPS_DATA_STRUCT
#include <iostream>
using namespace std;

// GPS 轨迹信息存储结构
typedef struct _gpsinfo{
	string id;
	string dateTime;
	string lon;
	string lat;
	string angle;

	_gpsinfo(){
		this->id;
		this->dateTime = "";
		this->lon = "";
		this->lat = "";
		this->angle = "";
	}
}GPSINFO;
#endif // ! 

