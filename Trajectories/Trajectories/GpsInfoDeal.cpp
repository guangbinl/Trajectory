#include "GpsDataStruct.h"
#include "GPSinfoDeal.h"
#include <fstream>
#include <sstream>
#include <vector>
#include <algorithm>
#include <Windows.h>


void  GpsinfoDeal::GetRoadName(const string& gps_file_full_path, string& result)
{
	result = "";
	vector<string> str_vec;
	str_vec.clear();
	CommonLibary::split(gps_file_full_path, "\\", str_vec);
	vector<string>::const_iterator iter = str_vec.begin();
	for (; iter != str_vec.end(); iter++){
		if (iter->find("：") != string::npos && iter->find("(") != std::string::npos && iter->find(")") != string::npos){
			string firstStr = iter->substr(0, iter->find("："));
			string secondStr = iter->substr(iter->find("(") + 1, iter->find(")") - iter->find("(") - 1);
			result = firstStr + "_" + secondStr;
			transform(result.begin(), result.end(), result.begin(), ::toupper);
		}
	}
	str_vec.clear();
}

void GpsinfoDeal::DealWithDateTime(string& datetime)
{
	string split_str = " ";
	vector<string> result;
	result.clear();
	CommonLibary::split(datetime, split_str, result);
	if (result.size() != 2){
		// error log
		return;
	}
	else{
		// delete "-"
		CommonLibary::Trim(result.at(0), '-');
		if (result.at(0).length() != 8){
			// error log
		}
		// delete ":"
		CommonLibary::Trim(result.at(1), ':');
		// add length 6 byte
		if (result.at(1).length() == 5){
			result.at(1).insert(0, "0");
		}
	}
	datetime = result.at(0) + result.at(1);
}
bool GpsinfoDeal::ReadGpsInfo(string&  gps_files_path)
{
	if (gps_files_path.empty())
		return false;

	// 获取当前目录下的所有文件名的名称
	vector<string> gps_file_list;
	CommonLibary::getAllFiles(gps_files_path, gps_file_list);
	if (gps_file_list.size() == 0)
		return false;

	vector<string>::const_iterator iter = gps_file_list.begin();
	for (; iter != gps_file_list.end(); iter++)
	{
		ifstream infile;
		infile.open(iter->c_str(), ios::in);
		if (!infile.is_open()){
			cout << "Open file is error, The file path = " << *iter << endl;
			continue;
		}

		map<string, GPSINFO> temp_map_gpsinfo;
		temp_map_gpsinfo.clear();

		string lineStr;
		while (getline(infile, lineStr)){
			//cout << lineStr << endl;
			stringstream ss(lineStr);
			string onefield;
			vector<string> lineArray;
			while (getline(ss, onefield, ',')){
				lineArray.push_back(onefield);
			}
			GPSINFO temp_gps_file;
			if (lineArray.size() == 7) // 5 field
			{
				temp_gps_file.id = lineArray.at(4); // id
				temp_gps_file.dateTime = lineArray.at(5)+' '+lineArray.at(6); // datetime
				DealWithDateTime(temp_gps_file.dateTime);
				temp_gps_file.lon = lineArray.at(1); // lon
				temp_gps_file.lat = lineArray.at(0); // lat
				//temp_gps_file.angle = lineArray.at(4); // angle
			}
			else{
				continue;
			}
			temp_map_gpsinfo.insert(make_pair(temp_gps_file.dateTime, temp_gps_file));
		}
		string road_name;
		m_gps_info.insert(make_pair(iter->c_str(), temp_map_gpsinfo));
		infile.close();
	}
	return true;
}
void GpsinfoDeal::getAllGpsPoint(VectorPoints& point_vec)
{
	std::map<string, map<string, GPSINFO>>::iterator  file_itr = m_gps_info.begin();
	for (; file_itr != m_gps_info.end(); file_itr++)
	{
		map<string, GPSINFO>::iterator point_itr= file_itr->second.begin();
		tagSpace point;
		for (; point_itr != file_itr->second.end(); point_itr++)
		{

			point.x = atof(point_itr->second.lon.c_str()) * MESH1_E_S * MESH_MIN_UNIT;
			point.y = atof(point_itr->second.lat.c_str()) * MESH1_E_S * MESH_MIN_UNIT;
			point.z = 0;

			point_vec.push_back(point);
		}
	}
}

void GpsinfoDeal::getEveryGpsPoint(map<string, VectorPoints>& point_vec)
{
	std::map<string, map<string, GPSINFO>>::iterator  file_itr = m_gps_info.begin();
	for (; file_itr != m_gps_info.end(); file_itr++)
	{
		map<string, GPSINFO>::iterator point_itr = file_itr->second.begin();
		tagSpace point;
		for (; point_itr != file_itr->second.end(); point_itr++)
		{

			point.x = atof(point_itr->second.lon.c_str()) * MESH1_E_S * MESH_MIN_UNIT;
			point.y = atof(point_itr->second.lat.c_str()) * MESH1_E_S * MESH_MIN_UNIT;
			point.z = 0;
			point.datateime = atoll(point_itr->second.dateTime.c_str());
			point_vec[file_itr->first].push_back(point);
		}
	}
}