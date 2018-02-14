/*
 * Objects.hpp
 *
 *  Created on: Mar 9, 2017
 *      Author: Kourosh Darvish
 */
#ifndef OBJECTS_HPP
#define OBJECTS_HPP

#include <stdlib.h>
#include <iostream>
#include <stdio.h>
#include <fstream>
#include <vector>
#include <string>
#include <math.h>
//#include <cmat/cmat.h>
//#include <Eigen/Dense>
//#include <Eigen/Geometry>
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
//using namespace Eigen;

namespace pittObjects {

class Frame{
public:
	vector<string> name;
	float frame[6];
	Frame(){
		for(int i=0;i<6;i++)
			frame[i]=0.0;
	}
	Frame(vector<string> Name, float *Frame){
		name=Name;
		for(int i=0;i<6;i++)
			frame[i]=Frame[i];
	}
	~Frame(){}
	void Print(void) const{
		for(int i=0;i<name.size();i++)
			cout<<name[i]<<" ";
		cout<<": ";
		for(int i=0;i<6;i++)
			cout<<frame[i]<<" ";
		cout<<endl;
	}
};

class Objects{
public:
//Variables:
	float obstacleSafetyFactor;  //! bounding ball   safety factor (1.5, 2.5)
	float obstacleSafetyFactorX; //! bounding box  x safety factor
	float obstacleSafetyFactorY; //! bounding box  y safety factor
	float obstacleSafetyFactorZ; //! bounding box  z safety factor

	string	 		objType;
	string			objName;
	int 			objID;
	TrackedShape 	trackedShape;
	vector<Frame>	objectFrames;
	float			boundingBall[4];// center: [x y z], radius
	float			boundingBox[6]; // center: [x y z], object size: [x y z]


	float 			objFrame[6]; //! [y,p,r,x,y,z]
//	CMAT::RotMatrix RotMat_W2Obj;
	Eigen::Matrix3f RotMat_World2Obj;
	float			objectGraspPose[6]; //! [y,p,r,x,y,z]
	float			objPathPlanningPose[6];//[y,p,r,x,y,z]
	float			GraspPoseDistance;

//Methods:
	Objects(const string obj_type,const int obj_id,const string obj_name,const TrackedShape& track_shape):objType(obj_type),
			objID(obj_id),trackedShape(track_shape),objName(obj_name){

		cout<<"Objects: "<<objType<<endl;
		for (int i=0;i<6;i++){
			objFrame[i]=0.0;
			objectGraspPose[i]=0.0;
			objPathPlanningPose[i]=0.0;
			boundingBox[i]=0.0;
		}

		for (int i=0;i<4;i++)
			boundingBall[i]=0.0;

		for (int i=0;i<3;i++)
			for (int j=0;j<3;j++){
//				RotMat_W2Obj(i,j)=0.0;
				RotMat_World2Obj(i,j)=0.0;
			}

		obstacleSafetyFactor=1.1;
		obstacleSafetyFactorX=1.1; //! bounding box  x safety factor
		obstacleSafetyFactorY=1.1; //! bounding box  y safety factor
		obstacleSafetyFactorZ=1.1; //! bounding box  z safety factor
		GraspPoseDistance=0.1;

	}

	Objects(const Objects& O){
		cout<<"Copy Constructor Objects: "<<O.objType<<endl;
		objType=O.objType;
		objID=O.objID;
		objName=O.objName;
		trackedShape=O.trackedShape;
		obstacleSafetyFactor=O.obstacleSafetyFactor;
		obstacleSafetyFactorX=O.obstacleSafetyFactorX;
		obstacleSafetyFactorY=O.obstacleSafetyFactorY;
		obstacleSafetyFactorZ=O.obstacleSafetyFactorZ;
		GraspPoseDistance=O.GraspPoseDistance;
		objectFrames=O.objectFrames;

		for (int i=0;i<6;i++){
			objFrame[i]=O.objFrame[i];
			objectGraspPose[i]=O.objectGraspPose[i];
			objPathPlanningPose[i]=O.objPathPlanningPose[i];
			boundingBox[i]=O.boundingBox[i];
		}

		for (int i=0;i<4;i++)
			boundingBall[i]=O.boundingBall[i];

		for (int i=0;i<3;i++)
			for (int j=0;j<3;j++){
//				RotMat_W2Obj(i,j)=O.RotMat_W2Obj(i,j);
				RotMat_World2Obj(i,j)=O.RotMat_World2Obj(i,j);
			}
	}

	virtual Objects& operator=(const Objects &O){ // ?? Virtual or not?
		if (&O!=this){
			objType=O.objType;
			objID=O.objID;
			objName=O.objName;
			trackedShape=O.trackedShape;
			obstacleSafetyFactor=O.obstacleSafetyFactor;
			obstacleSafetyFactorX=O.obstacleSafetyFactorX;
			obstacleSafetyFactorY=O.obstacleSafetyFactorY;
			obstacleSafetyFactorZ=O.obstacleSafetyFactorZ;
			GraspPoseDistance=O.GraspPoseDistance;
			objectFrames=O.objectFrames;

			for (int i=0;i<6;i++)
			{
				objFrame[i]=O.objFrame[i];
				objectGraspPose[i]=O.objectGraspPose[i];
				objPathPlanningPose[i]=O.objPathPlanningPose[i];
				boundingBox[i]=O.boundingBox[i];
			}

			for (int i=0;i<4;i++)
				boundingBall[i]=O.boundingBall[i];

			for (int i=0;i<3;i++)
				for (int j=0;j<3;j++)
				{
//					RotMat_W2Obj(i,j)=O.RotMat_W2Obj(i,j);
					RotMat_World2Obj(i,j)=O.RotMat_World2Obj(i,j);
				}
		}
		cout<<"Objects::operator=(const Objects &)"<<endl;
		return *this;
	}

	virtual ~Objects(){
		cout<<"~Objects()"<<endl;
	}

	void Print() const{
		cout<<FGRN("***** Objects::Print() *****")<<endl;
		cout<<"Object Type: "<<objType<<endl;
		cout<<"Object Name: "<<objName<<endl;
		cout<<"Object ID: "<<objID<<endl;

		cout<<"Object Center: "<<trackedShape.x_pc_centroid<< " "<<trackedShape.y_pc_centroid<< " "<<trackedShape.z_pc_centroid<< " "<<endl;

		cout<<"Object Bounding Box: ";
		for(int i=0;i<6;i++) cout<<boundingBox[i]<<" ";
		cout<<endl;

		cout<<"Object Bounding Ball: ";
		for(int i=0;i<4;i++) cout<<boundingBall[i]<<" ";
		cout<<endl;

		cout<<"Object Frames:"<<endl;
		for(int i=0;i< objectFrames.size();i++)
			objectFrames[i].Print();
	}

	virtual void BoundingBox(void){};
	virtual void BoundingBall(void){};
	virtual void GraspingPosition(void){};//! Define the grasping and approaching position for robot
	virtual void FrameSet(void){};
};
// =======================================

class Sphere:public Objects{
public:
	Sphere(const int obj_id,const string obj_name,const TrackedShape& track_shape):Objects("sphere",obj_id,obj_name,track_shape){}
	Sphere(const Sphere & S):Objects(S){}
	Sphere& operator=(const Sphere &S){ // ?? Virtual or not?
		if (&S!=this){
			this->Objects::operator =(S);
		}
		return *this;
	}
	void BoundingBox(void);
	void BoundingBall(void);
	void GraspingPosition(void);
	void FrameSet(void);
};

// =======================================

class Cylinder:public Objects{
public:
	Cylinder(const int obj_id,const string obj_name, const TrackedShape& track_shape):Objects("cylinder",obj_id,obj_name,track_shape){}
	Cylinder(const Cylinder & C):Objects(C){}
	Cylinder& operator=(const Cylinder &C){ // ?? Virtual or not?
		if (&C!=this){
			this->Objects::operator =(C);
		}
		return *this;
	}
	void BoundingBox(void);
	void BoundingBall(void);
	void GraspingPosition(void);
	void FrameSet(void);
//	INFO:
//	coefficients: [Point A, Norm of Axis of Cylinder,Radius,Height]
// 	Point A: a point belong to axis of the cylinder [x,y,z] (meter)	[0-2]
// 	Norm of Axis of Cylinder: NormAxis=[nx,ny,nz] (meter)			[3-5]
// 	Radius: (meter)													[6]
// 	Height: (meter)													[7]

//	    x_est_centroid: X center on the cylinder axis
//	    y_est_centroid: Y center on the cylinder axis
//	    z_est_centroid: Z center on the cylinder axis

};
// =======================================
class Plane:public Objects{
public:
	Plane(const int obj_id,const string obj_name,const TrackedShape& track_shape):Objects("plane",obj_id,obj_name,track_shape){}
	Plane(const Plane & P):Objects(P){}
	Plane& operator=(const Plane& P){ // ?? Virtual or not?
		if (&P!=this){
			this->Objects::operator =(P);
		}
		return *this;
	}
	void BoundingBox(void);
	void BoundingBall(void);
	void GraspingPosition(void);
	void FrameSet(void);
};
// =======================================
// =======================================
class Cone:public Objects{
public:
	Cone(const int obj_id,const string obj_name,const TrackedShape& track_shape):Objects("cone",obj_id,obj_name,track_shape){}
	Cone(const Cone & C):Objects(C){}
	Cone& operator=(const Cone& C){ // ?? Virtual or not?
		if (&C!=this){
			this->Objects::operator =(C);
		}
		return *this;
	}
	void BoundingBox(void);
	void BoundingBall(void);
	void GraspingPosition(void);
	void FrameSet(void);
};
// =======================================
class Unknown:public Objects{
public:
	Unknown(const int obj_id,const string obj_name,const TrackedShape& track_shape):Objects("unknown",obj_id,obj_name,track_shape){}
	Unknown(const Unknown & U):Objects(U){}
	Unknown& operator=(const Unknown& U){ // ?? Virtual or not?
		if (&U!=this){
			this->Objects::operator =(U);
		}
		return *this;
	}
	void BoundingBox(void);
	void BoundingBall(void);
	void GraspingPosition(void);
	void FrameSet(void);
};
// =======================================

}  // namespace Objects



#endif


