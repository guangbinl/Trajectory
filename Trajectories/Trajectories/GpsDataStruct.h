/**************************************************************************************
 GPS �켣��ؽṹ�Ķ���
 **************************************************************************************/
#ifndef   GPS_DATA_STRUCT
#define  GPS_DATA_STRUCT
#include <iostream>
using namespace std;

// GPS �켣��Ϣ�洢�ṹ
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

