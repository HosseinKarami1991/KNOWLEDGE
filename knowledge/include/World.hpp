/*
 * Objects.hpp
 *
 *  Created on: Mar 9, 2017
 *      Author: Kourosh Darvish
 */
#ifndef WORLD_HPP
#define WORLD_HPP

#include <stdlib.h>
#include <iostream>
#include <stdio.h>
#include <fstream>
#include <vector>
#include <string>
#include <math.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>



#include "pitt_msgs/TrackedShape.h"
#include "pitt_msgs/TrackedShapes.h"

typedef ::pitt_msgs::TrackedShape_<std::allocator<void> > TrackedShape;
typedef ::pitt_msgs::TrackedShapes_<std::allocator<void> > TrackedShapes;

#define RST  "\x1B[0m"
#define KGRN  "\x1B[32m"
#define FGRN(x) KGRN x RST

using namespace pitt_msgs;
using namespace std;

class World{
public:
	vector<string> name;
	vector<float> value;
	World(void){}
	World(vector<string> Name, vector<float> Value){
		name=Name;
		value=Value;
	}

	~World(){}
	void Print(void ){
		cout<<"name: ";
		for(int i=0;i<name.size();i++)
			cout<<name[i]<<" ";
		cout<<endl;
		cout<<"value: ";

		for(int i=0;i<value.size();i++)
			cout<<value[i]<<" ";
		cout<<endl;
	}
};

#endif
