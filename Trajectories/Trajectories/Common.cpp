#include "Common.h"

void CommonLibary::Trim(string& source, char trim_char){
	string::iterator iter = source.begin();
	for (; iter != source.end(); iter++)
	{
		if (*iter == trim_char)
		{
			source.erase(iter);
		}
	}
}

void CommonLibary::split(const string &source, const string &pattern, vector<string>& result)
{
	result.clear();
	if (source.empty()){
		return;
	}
	//方便截取最后一段数据
	string temp_str = source + pattern;

	size_t pos = temp_str.find(pattern);
	size_t size = temp_str.size();
	while (pos != std::string::npos){
		std::string x = temp_str.substr(0, pos);
		result.push_back(x);
		temp_str = temp_str.substr(pos + 1, size);
		pos = temp_str.find(pattern);
	}
}



// 度分秒转换为弧度
void CoordinateConvert::DMS_RAD(double DMS, double *Rad) {
	*Rad = DMS / 180.0*PI;
	return;
}


// 弧度转换为度分秒
void CoordinateConvert::RAD_DMS(double Rad, double *DMS) {
	int Deg, Min;
	double Sec;
	double AR, AM;
	AR = Rad;
	if (Rad < 0)
		AR = -Rad;
	AR = AR + 1.0e-10;
	AR = AR * 180.0 / PI;
	Deg = (int)AR;
	AM = (AR - Deg)*60.0;
	Min = (int)AM;
	Sec = (AM - Min) * 60;
	*DMS = Deg + Min / 100.0 + Sec / 10000.0;
	if (Rad < 0)
		*DMS = -*DMS;
	return;
}

// 空间直角坐标系转换为大地坐标系
bool CoordinateConvert::Space2Ground(pSpace pS, pGround pG, double a, double f) {
	double B0, R, N;
	double B_, L_;
	double X = pS->x;
	double Y = pS->y;
	double Z = pS->z;

	double b = a - f * a;
	double e = sqrt((a*a - b * b) / (a * a));

	R = sqrt(X*X + Y * Y);
	B0 = atan2(Z, R);
	while (1) {
		N = a / sqrt(1.0 - e * e*sin(B0)*sin(B0));
		B_ = atan2(Z + N * e*e*sin(B0), R);
		if (fabs(B_ - B0) < 1.0e-10)
			break;
		B0 = B_;
	}
	L_ = atan2(Y, X);

	pG->height = R / cos(B_) - N;
	RAD_DMS(B_, &pG->latitude);
	RAD_DMS(L_, &pG->longitude);

	return true;
}

// 大地坐标系转换为空间直角坐标系
bool CoordinateConvert::Ground2Space(pGround pG, pSpace pS, double a, double f) {
	double N, B_, L_;
	double B = pG->latitude;
	double L = pG->longitude;
	double H = pG->height;

	double b = a - f * a;
	double e = sqrt((a*a - b * b) / (a * a));

	DMS_RAD(B, &B_);
	DMS_RAD(L, &L_);

	N = a / sqrt(1.0 - e*e*sin(B_)*sin(B_));
	pS->x = (N + H)*cos(B_)*cos(L_);
	pS->y = (N + H)*cos(B_)*sin(L_);
	pS->z = (N*(1.0 - e*e) + H)*sin(B_);
	return true;
}

void CommonLibary::getAllFiles(string path, vector<string>& files)
{
	//文件句柄 
	long  hFile = 0;
	//文件信息 
	struct _finddata_t fileinfo;
	string p;
	if ((hFile = _findfirst(p.assign(path).append("\\*").c_str(), &fileinfo)) != -1)
	{
		do
		{
			if ((fileinfo.attrib & _A_SUBDIR))
			{
				if (strcmp(fileinfo.name, ".") != 0 && strcmp(fileinfo.name, "..") != 0)
				{
					files.push_back(p.assign(path).append("\\").append(fileinfo.name));
					getAllFiles(p.assign(path).append("\\").append(fileinfo.name), files);
				}
			}
			else
			{
				files.push_back(p.assign(path).append("\\").append(fileinfo.name));
			}
		} while (_findnext(hFile, &fileinfo) == 0);
		_findclose(hFile);
	}
}
