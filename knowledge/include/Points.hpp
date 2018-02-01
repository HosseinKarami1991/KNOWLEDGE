#include <stdlib.h>
#include <iostream>
#include <stdio.h>
#include <fstream>
#include <vector>
#include <string>
#include <math.h>
#include <boost/algorithm/string.hpp>
#include "World.hpp"

#define RST  "\x1B[0m"
#define KBLU  "\x1B[34m"
#define KRED  "\x1B[31m"
#define KGRN  "\x1B[32m"
#define FBLU(x) KBLU x RST
#define FRED(x) KRED x RST
#define FGRN(x) KGRN x RST
#define BOLD(x) "\x1B[1m" x RST


using namespace std;


//class Point{
//public:
//	string name;
//	vector<float> pose;
//	Point(string Name, vector<float> Pose){
//		name=Name;
//		for (int i=0;i<Pose.size();i++)
//		{
//			pose.push_back(Pose[i]);
//		}
//	}
//	~Point(){}
//	void Print(void){
//		cout<<"******* Point::Print()  ******** "<<name<<endl;
//
//		cout<<"name: "<<name<<endl;
//		cout<<"pose: ";
//		for (int i=0;i<pose.size();i++)
//		{
//			cout<<pose[i]<<" ";
//		}
//		cout<<endl;
//	}
//};
//*****************************************
//*****************************************

void readPointsVector(string pointsPath, vector<World> &pointsVector){
	ifstream file_path_ifStr(pointsPath.c_str());
	string line;
	vector<string> line_list;

	string delim_type=" ";
	if (file_path_ifStr.is_open())
	{
		while(getline(file_path_ifStr,line))
		{
			boost::split(line_list, line, boost::is_any_of(delim_type));
			if(line_list[0]!="#")
			{
				vector<float> Pose;
				vector<string> Name;
				for(int i=1;i<line_list.size();i++)
				{
					Pose.push_back( stof(line_list[i]) );
				}

				Name.push_back(line_list[0]);
				Name.push_back("Pose");
				World temp_point(Name,Pose);
				pointsVector.push_back(temp_point);
			}
		}
	}

	for(int i=0;i<pointsVector.size();i++)
		pointsVector[i].Print();
};

