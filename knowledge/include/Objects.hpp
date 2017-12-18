/*
 * Objects.hpp
 *
 *  Created on: Mar 9, 2017
 *      Author: Kourosh Darvish
 */
#ifndef DOCUMENT_HPP
#define DOCUMENT_H

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

using namespace pitt_msgs;
using namespace std;
//using namespace Eigen;

namespace pittObjects {

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
		}
		for (int i=0;i<3;i++)
			for (int j=0;j<3;j++){
//				RotMat_W2Obj(i,j)=0.0;
				RotMat_World2Obj(i,j)=0.0;
			}

		obstacleSafetyFactor=1.0;
		obstacleSafetyFactorX=1.0; //! bounding box  x safety factor
		obstacleSafetyFactorY=1.0; //! bounding box  y safety factor
		obstacleSafetyFactorZ=1.0; //! bounding box  z safety factor
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
		for (int i=0;i<6;i++){
			objFrame[i]=O.objFrame[i];
			objectGraspPose[i]=O.objectGraspPose[i];
			objPathPlanningPose[i]=O.objPathPlanningPose[i];
		}
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
			for (int i=0;i<6;i++){
				objFrame[i]=O.objFrame[i];
				objectGraspPose[i]=O.objectGraspPose[i];
				objPathPlanningPose[i]=O.objPathPlanningPose[i];
			}
			for (int i=0;i<3;i++)
				for (int j=0;j<3;j++){
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
		cout<<"Objects::Print()"<<endl;
		cout<<"Object Type: "<<objType<<endl;
		cout<<"Object ID: "<<objID<<endl;
		cout<<"Object Name: "<<objName<<endl;

		cout<<"Object Center: "<<trackedShape.x_pc_centroid<< " "<<trackedShape.y_pc_centroid<< " "<<trackedShape.z_pc_centroid<< " "<<endl;

	}

	virtual void BoundingBox(float *){};
	virtual void BoundingBall(float *){};
	virtual void GraspingPosition(float * graspPose,float * pathPlanningPose,string graspingPosDef){};//! Define the grasping and approaching position for robot
	virtual void FrameSet(void){};
	virtual int RobotResponsibleArm(void){
		/*! for single arm grasping for the objects:
		 * the distance of the object center from each arm shoulder defines which arm grasp the object
		 * 		the shoulder closer to object center will grasp it
		 * 		it can be proved that the less distance can be defined by just checking the y value of the center
		 * 		if y>0 --> left arm
		 * 		if y<0 --> right arm
		 *
		 * */

		return 0;};
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
	void BoundingBox(float *);
	void BoundingBall(float *);
	void GraspingPosition(float * graspPose,float * pathPlanningPose,string graspingPosDef);
	void FrameSet(void);
	int RobotResponsibleArm(void);
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
	void BoundingBox(float *);
	void BoundingBall(float *);
	void GraspingPosition(float * graspPose,float * pathPlanningPose,string graspingPosDef);
	void FrameSet(void);
	int RobotResponsibleArm(void);
//	coefficients: [Point A, Norm of Axis of Cylinder,Radius,Height]
	// Point A: a point belong to axis of the cylinder [x,y,z] (meter)	[0-2]
	// Norm of Axis of Cylinder: NormAxis=[nx,ny,nz] (meter)			[3-5]
	// Radius: (meter)													[6]
	// Height: (meter)													[7]

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
	void BoundingBox(float *);
	void BoundingBall(float *);
	void GraspingPosition(float * graspPose,float * pathPlanningPose,string graspingPosDef);
	void FrameSet(void);
	int RobotResponsibleArm(void);
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
	void BoundingBox(float *);
	void BoundingBall(float *);
	void GraspingPosition(float * graspPose,float * pathPlanningPose,string graspingPosDef);
	void FrameSet(void);
	int RobotResponsibleArm(void);
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
	void BoundingBox(float *);
	void BoundingBall(float *);
	void GraspingPosition(float * graspPose,float * pathPlanningPose ,string graspingPosDef);
	void FrameSet(void);
	int RobotResponsibleArm(void);
};
// =======================================

}  // namespace Objects

class Point{
public:
	string name;
	vector<float> pose;
	Point(string Name, vector<float> Pose){
		name=Name;
		for (int i=0;i<Pose.size();i++)
		{
			pose.push_back(Pose[i]);
		}
	}
	~Point(){}
	void Print(void){
		cout<<"******* Point::Print()  ******** "<<name<<endl;

		cout<<"name: "<<name<<endl;
		cout<<"pose: ";
		for (int i=0;i<pose.size();i++)
		{
			cout<<pose[i]<<" ";
		}
		cout<<endl;
	}
};



#endif


