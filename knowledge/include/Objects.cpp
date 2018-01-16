/*
 * Objects.cpp
 *
 *  Created on: Mar 10, 2017
 *      Author: Kourosh Darvish
 */

#include "Objects.hpp"

void pittObjects::Sphere::BoundingBox(void){
	cout<<"Sphere::BoundingBox"<<endl;
	boundingBox[0]=trackedShape.x_est_centroid; //! center_x
	boundingBox[1]=trackedShape.y_est_centroid;//! center_y
	boundingBox[2]=trackedShape.z_est_centroid;//! center_z

	boundingBox[3]=trackedShape.coefficients[3]*2.0*obstacleSafetyFactorX;//! size_x,
	boundingBox[4]=trackedShape.coefficients[3]*2.0*obstacleSafetyFactorY;//! size_y
	boundingBox[5]=trackedShape.coefficients[3]*2.0*obstacleSafetyFactorZ;//! size_z
}

void pittObjects::Sphere::BoundingBall(void){
	cout<<"Sphere::BoundingBall"<<endl;
	boundingBall[0]=trackedShape.x_est_centroid; //! center_x
	boundingBall[1]=trackedShape.y_est_centroid;//! center_y
	boundingBall[2]=trackedShape.z_est_centroid;//! center_z
	boundingBall[3]=trackedShape.coefficients[3]*2.0*obstacleSafetyFactor;//! size_x,
}

void pittObjects::Sphere::GraspingPosition(void){
	cout<<"Sphere::GraspingPosition"<<endl;
	/*! Grasping Pose is computed from top of the sphere considering
	 * x axis object = x axis gripper ~ PI (3.14) : Roll
	 * y axis object = y axis gripper~ 0 : Pitch
	 * z axis object = z axis gripper~ 0 : Yaw
	 * x center object grasp= x position gripper
	 * y center object grasp= y position gripper
	 * z center object grasp= z position gripper
	 *  */

	float graspPose[6]; float approachingPose[6];
	//RM_: rotation matrix;
	// object CenterFrame rotation matrix, which is parallel to the world frame for the sphere
	Eigen::Matrix3f RM_World2Object;
	for (int i=1;i<4;i++)
		for (int j=1;j<4;j++)
			if (i==j)
				RM_World2Object(i-1,j-1)=1.0;
			else
				RM_World2Object(i-1,j-1)=0.0;

	Eigen::Vector3f Vec_ObjectFr2GraspFr, grasping_EulerAngles; // YPR
	Vec_ObjectFr2GraspFr<< 3.14, 0.0, 3.14;
	Eigen::Matrix3f ROtMat_ObjectFr2GraspFr, RotMat_world2Grasping;
	ROtMat_ObjectFr2GraspFr = Eigen::AngleAxisf(Vec_ObjectFr2GraspFr(0), Eigen::Vector3f::UnitZ())
	* Eigen::AngleAxisf(Vec_ObjectFr2GraspFr(1), Eigen::Vector3f::UnitY())
	* Eigen::AngleAxisf(Vec_ObjectFr2GraspFr(2), Eigen::Vector3f::UnitX());

	RotMat_world2Grasping= RM_World2Object*ROtMat_ObjectFr2GraspFr;
	grasping_EulerAngles=RotMat_world2Grasping.eulerAngles(2,1,0);
	cout<<"Eigen::grasping_EulerAngles:\n"<<grasping_EulerAngles<<endl;

	float graspingPose[6];
	graspingPose[0]=grasping_EulerAngles(0); //Y
	graspingPose[1]=grasping_EulerAngles(1);  //P
	graspingPose[2]=grasping_EulerAngles(2);  //R
	graspingPose[3]=objFrame[3];
	graspingPose[4]=objFrame[4];
	graspingPose[5]=objFrame[5];

	class Frame tempGrapsingFrame("graspingPose1",graspingPose);
	objectFrames.push_back(tempGrapsingFrame);


	approachingPose[0]=graspPose[0]; //Y
	approachingPose[1]=graspPose[1];// P
	approachingPose[2]=graspPose[2];// R

	approachingPose[3]=graspPose[3]-RotMat_world2Grasping(0,2)*GraspPoseDistance;
	approachingPose[4]=graspPose[4]-RotMat_world2Grasping(1,2)*GraspPoseDistance;
	approachingPose[5]=graspPose[5]-RotMat_world2Grasping(2,2)*GraspPoseDistance;

	class Frame tempApproachingFrame("approachingPose1",approachingPose);
	objectFrames.push_back(tempApproachingFrame);

}

void pittObjects::Sphere::FrameSet(void){
	cout<<"Sphere::FrameSet"<<endl;
	cout<<"Frame=> parallel to World Frame"<<endl;

	float tempObjFrame[]={0.0, 0.0, 0.0,trackedShape.x_est_centroid,trackedShape.y_est_centroid,trackedShape.z_est_centroid };
	class Frame tempFrame("centerFrame",tempObjFrame);
	objectFrames.push_back(tempFrame);

}

// =======================================

void pittObjects::Cylinder::BoundingBox(void){
	cout<<"Cylinder::BoundingBox"<<endl;
	/*!
	 * rrt* method accept boxes just parallel to x,y,z axis of the world
	 * cylinder can be in any direction, based on its principal axis
	 * it is necessary to find a bounding box that is parallel to world axis and encompass the cylinder in all the direction.
	 * consider that the cylinder is symmetrical
	 * map on axis i:
	 * 				m_i=h/2*n_i+r*r_i=h/2*cos(theta)+h/2*cos(90-theta)=h/2*cos(theta)+h/2*sqrt(1-sin(theta)^2)
	 * */

	float height, radius, normalAxis[3];
	radius= trackedShape.coefficients[6];
	height= trackedShape.coefficients[7];
	normalAxis[0]=trackedShape.coefficients[3];
	normalAxis[1]=trackedShape.coefficients[4];
	normalAxis[2]=trackedShape.coefficients[5];

	boundingBox[0]=trackedShape.x_est_centroid;//! center_x
	boundingBox[1]=trackedShape.y_est_centroid;//! center_y
	boundingBox[2]=trackedShape.z_est_centroid;//! center_z

	boundingBox[3]=	(height/2.0)* normalAxis[0]+radius*sqrt(1-normalAxis[0]*normalAxis[0]);
	boundingBox[4]=	(height/2.0)* normalAxis[1]+radius*sqrt(1-normalAxis[1]*normalAxis[1]);
	boundingBox[5]=	(height/2.0)* normalAxis[2]+radius*sqrt(1-normalAxis[2]*normalAxis[2]);

	boundingBox[3]=boundingBox[3]*2.0*obstacleSafetyFactorX;//! size_x,
	boundingBox[4]=boundingBox[4]*2.0*obstacleSafetyFactorY;//! size_y,
	boundingBox[5]=boundingBox[5]*2.0*obstacleSafetyFactorZ;//! size_z,

}

void pittObjects::Cylinder::BoundingBall(void){
	cout<<"Cylinder::BoundingBall"<<endl;
	float height, radius, normalAxis[3], ballRadius;
	radius= trackedShape.coefficients[6];
	height= trackedShape.coefficients[7];
	normalAxis[0]=trackedShape.coefficients[3];
	normalAxis[1]=trackedShape.coefficients[4];
	normalAxis[2]=trackedShape.coefficients[5];

	ballRadius=sqrt(height*height/4+radius*radius);

	boundingBall[0]=trackedShape.x_est_centroid;//! center_x
	boundingBall[1]=trackedShape.y_est_centroid;//! center_y
	boundingBall[2]=trackedShape.z_est_centroid;//! center_z
	boundingBall[3]=ballRadius*2.0*obstacleSafetyFactor;//! Diameter,

}

void pittObjects::Cylinder::GraspingPosition(void){
	cout<<"Cylinder::GraspingPosition"<<endl;

	float graspPose[6]; float approachingPose[6];

	Eigen::Vector3f Vec_ObjFr2GraspFr, vec_Grasping; //YPR
	Eigen::Matrix3f Rot_Obj2Grasp, Rot_Grasping,RotMat;
	float height, radius, normalAxis[3];


	/*
	Vec_ObjFr2GraspFr(1)=0.0; Vec_ObjFr2GraspFr(2)=0.0; Vec_ObjFr2GraspFr(3)=3.14;
	Rot_Obj2Grasp=Vec_ObjFr2GraspFr.Vect2RPY();
	Rot_Obj2Grasp.PrintMtx("Rot_Obj2Grasp: ");
	Rot_Grasping= RotMat_W2Obj*Rot_Obj2Grasp;

	Rot_Grasping.PrintMtx("Rotation matrix Grasping: ");
	vec_Grasping= Rot_Grasping.RPY2vect();
	vec_Grasping.Transpose().PrintMtx("Grasping Vector (Y -P R): ");

	graspPose[0]=vec_Grasping(1); //Y
	graspPose[1]=-vec_Grasping(2);  //P
	graspPose[2]=vec_Grasping(3);  //R
	graspPose[3]=objFrame[3];
	graspPose[4]=objFrame[4];
	graspPose[5]=objFrame[5];
	cout<<"Grasping vector(1): "<<endl;
	for (int i=0;i<6;i++)
		cout<<graspPose[i]<<" ";
	cout<<endl;

	cout<<"********"<<endl;
	 */



	radius= trackedShape.coefficients[6];
	height= trackedShape.coefficients[7];
	normalAxis[0]=trackedShape.coefficients[3];
	normalAxis[1]=trackedShape.coefficients[4];
	normalAxis[2]=trackedShape.coefficients[5];
	cout<<"-------------------"<<endl;
	cout<<"RADIUS:"<<radius<<" HEIGHT:"<<height<<endl;
	cout<<"-------------------"<<endl;
	float VERTICAL_THRESHOLD=0.99;
	float VERTICAL_GRASPING_POINT_THRESHOLD=0.05;// 5 cm from top we grasp the object
	/*!
	 * o if the cylinder is vertical:
	 * 	- vertical cylinder means normalAxis[2]>0.99
	 * 		in this case we grasp the cylinder from top like a sphere from top
	 * 	- we grasp it 5 cm from top
	 *
	 * o if the cylinder is not vertical:
	 *  - non vertical cylinder means normalAxis[2]<0.99
	 * 	- we should do some experiments and check for checking sign of biggest normal value, being positive or negative
	 * 		and its relation to success of grasping.
	 *
	 *
	 * */
	if (abs(normalAxis[2])> VERTICAL_THRESHOLD)
	{
		if (normalAxis[2]<0)
			for (int i=0;i<3;i++)
				normalAxis[i]=normalAxis[i]*(-1.0);

		graspPose[0]=3.14; //Y
		graspPose[1]=0.0;  //P
		graspPose[2]=3.14;  //R

		graspPose[3]=objFrame[3]+normalAxis[0]*(height/2.0 - VERTICAL_GRASPING_POINT_THRESHOLD);
		graspPose[4]=objFrame[4]+normalAxis[1]*(height/2.0 - VERTICAL_GRASPING_POINT_THRESHOLD);
		graspPose[5]=objFrame[5]+normalAxis[2]*(height/2.0 - VERTICAL_GRASPING_POINT_THRESHOLD);

		approachingPose[0]=graspPose[0]; //Y
		approachingPose[1]=graspPose[1];// P
		approachingPose[2]=graspPose[2];// R

		approachingPose[3]=graspPose[3]+normalAxis[0]*GraspPoseDistance;
		approachingPose[4]=graspPose[4]+normalAxis[1]*GraspPoseDistance;
		approachingPose[5]=graspPose[5]+normalAxis[2]*GraspPoseDistance;
	}

	else
	{
		Eigen::Vector3f RefPoint,ObjPoint;
		Eigen::Vector3f X_grasp, zPrime,Y_grasp,Z_grasp, EulerAngles;
		ObjPoint(0)=objFrame[3];ObjPoint(1)= objFrame[4];ObjPoint(2)= objFrame[5];
		RefPoint(0)=0.0; RefPoint(1)=0.0; RefPoint(2)=5.0;

		if (normalAxis[0]>0)
			for (int i=0;i<3;i++)
				normalAxis[i]=normalAxis[i]*(-1.0);

		X_grasp(0)=normalAxis[0];
		X_grasp(1)=normalAxis[1];
		X_grasp(2)=normalAxis[2];

		X_grasp=X_grasp/X_grasp.norm();

		zPrime=(ObjPoint- RefPoint);
		cout<<"zPrime: \n"<<zPrime<<endl;

		zPrime=zPrime/(zPrime.norm());
		cout<<"unit zPrime: \n"<<zPrime<<endl;

		Z_grasp=zPrime-X_grasp*(zPrime.dot(X_grasp));

		Z_grasp=Z_grasp/(Z_grasp.norm());
		cout<<"Z_grasp: \n"<<Z_grasp<<endl;


		Y_grasp=Z_grasp.cross(X_grasp);
		cout<<"Y_grasp: \n"<<Y_grasp<<endl;


		RotMat(0,0)=X_grasp(0); RotMat(0,1)=Y_grasp(0); RotMat(0,2)=Z_grasp(0);
		RotMat(1,0)=X_grasp(1); RotMat(1,1)=Y_grasp(1); RotMat(1,2)=Z_grasp(1);
		RotMat(2,0)=X_grasp(2); RotMat(2,1)=Y_grasp(2); RotMat(2,2)=Z_grasp(2);

		cout<<"RotMat: \n"<<RotMat<<endl;

		EulerAngles=RotMat.eulerAngles(2,1,0);

		cout<<"Euler Angles: \n"<<EulerAngles<<endl;

		graspPose[0]=EulerAngles(0); //Y
		graspPose[1]=EulerAngles(1);  //P
		graspPose[2]=EulerAngles(2);  //R
		graspPose[3]=objFrame[3];
		graspPose[4]=objFrame[4];
		graspPose[5]=objFrame[5];
		cout<<"Grasping vector: "<<endl;
		for (int i=0;i<6;i++)
			cout<<graspPose[i]<<" ";
		cout<<endl;

		approachingPose[0]=graspPose[0]; //Y
		approachingPose[1]=graspPose[1];// P
		approachingPose[2]=graspPose[2];// R

		approachingPose[3]=graspPose[3]-RotMat(0,2)*GraspPoseDistance;
		approachingPose[4]=graspPose[4]-RotMat(1,2)*GraspPoseDistance;
		approachingPose[5]=graspPose[5]-RotMat(2,2)*GraspPoseDistance;
	}

	class Frame temp_graspingFrame("graspingPose1",graspPose);
	class Frame temp_approachingFrame("approachingPose1",approachingPose);
	objectFrames.push_back(temp_graspingFrame);
	objectFrames.push_back(temp_approachingFrame);

}

void pittObjects::Cylinder::FrameSet(void){
	cout<<"Cylinder::FrameSet"<<endl;
	/*! we assign cylinder axis as x_cylinder
	 * 	We find the rotation matrix from x_world To x_cylinder= Rx.
	 * 	We rotate y, z axis of the world to y_cylinder,z_cylinder using Rx.
	 *
	 * */

	Eigen::Vector3f X_world, Y_world, Z_world, X_cylinder, Y_cylinder , Z_cylinder, EulerAngles,X_cross;
	double x_cosine;	//! cosine of angle between x_world, x_cylinder, inner product
	double x_sine;	//! sine of angle between x_world, x_cylinder

	Eigen::Matrix3f R, X_cross_skewSym, I,X_cross_skewSym2;
	X_world(0)=1.0; X_world(1)=0.0; X_world(2)=0.0;
	Y_world(0)=0.0; Y_world(1)=1.0; Y_world(2)=0.0;
	Z_world(0)=0.0; Z_world(1)=0.0; Z_world(2)=1.0;

	X_cylinder(0)=trackedShape.coefficients[3];
	X_cylinder(1)=trackedShape.coefficients[4];
	X_cylinder(2)=trackedShape.coefficients[5];

	X_cross=X_world.cross(X_cylinder); //check
	x_sine=X_cross.norm(); //check
	x_cosine=X_world.dot(X_cylinder);

	X_cross_skewSym(0,0)= 0.0;			X_cross_skewSym(0,1)= -X_cross(2);	X_cross_skewSym(0,2)= X_cross(1);
	X_cross_skewSym(1,0)= X_cross(2);	X_cross_skewSym(1,1)= 0.0;			X_cross_skewSym(1,2)= -X_cross(0);
	X_cross_skewSym(2,0)= -X_cross(1);	X_cross_skewSym(2,1)= X_cross(0);	X_cross_skewSym(2,2)= 0.0;
	//	I(1,1)= 1.0; I(1,2)= 0.0; I(1,3)= 0.0;
	//	I(2,1)= 0.0; I(2,2)= 1.0; I(2,3)= 0.0;
	//	I(3,1)= 0.0; I(3,2)= 0.0; I(3,3)= 1.0;
	I.setIdentity();

	if (x_cosine==-1.0)
	{
		R(0,0)=-1.0; R(0,1)= 0.0; R(0,2)= 0.0;
		R(1,0)= 0.0; R(1,1)=-1.0; R(1,2)= 0.0;
		R(2,0)= 0.0; R(2,1)= 0.0; R(2,2)= 1.0;
	}
	else
	{

		X_cross_skewSym2=X_cross_skewSym*X_cross_skewSym;
		for (int i=0;i<3;i++)
			for (int j=0;j<3;j++)
				X_cross_skewSym2(i,j)=X_cross_skewSym2(i,j)*((1.0-x_cosine)/(x_sine*x_sine));

		R=I+X_cross_skewSym+ X_cross_skewSym2;
	}

	EulerAngles=R.eulerAngles(2,1,0);; //[yaw, pitch, roll]

	float objFrame[6];
	RotMat_World2Obj=R;
	objFrame[0]=EulerAngles(0); // Y
	objFrame[1]=EulerAngles(1);// P
	objFrame[2]=EulerAngles(2); // R
	objFrame[3]=trackedShape.x_est_centroid;
	objFrame[4]=trackedShape.y_est_centroid;
	objFrame[5]=trackedShape.z_est_centroid;

	class Frame temp_centerFrame("centerFrame",objFrame);
	objectFrames.push_back(temp_centerFrame);

}

// =======================================

void pittObjects::Cone::BoundingBox(void){
	cout<<"Cone::BoundingBox"<<endl;

}

void pittObjects::Cone::BoundingBall(void){
	cout<<"Cone::BoundingBall"<<endl;
}

void pittObjects::Cone::GraspingPosition(void){
	cout<<"Cone::GraspingPosition"<<endl;
	float graspPose[6]; float approachingPose[6];

}

void pittObjects::Cone::FrameSet(void){
	cout<<"Cone::FrameSet"<<endl;


}

// =======================================
void pittObjects::Plane::BoundingBox(void){
	cout<<"Plane::BoundingBox"<<endl;

}

void pittObjects::Plane::BoundingBall(void){
	cout<<"Plane::BoundingBall"<<endl;
}

void pittObjects::Plane::GraspingPosition(void){
	cout<<"Plane::GraspingPosition"<<endl;
	float graspPose[6];float approachingPose[6];
}

void pittObjects::Plane::FrameSet(void){
	cout<<"Plane::FrameSet"<<endl;
	float teta;
	float normalVec[3];
	//	float worldvector[3];
	//	float planeSize[2];
	normalVec[0]=trackedShape.coefficients[0];//a
	normalVec[1]=trackedShape.coefficients[1];//b
	normalVec[2]=trackedShape.coefficients[2];//c
	//	worldvector[0]=0.0;
	//	worldvector[1]=0.0;
	//	worldvector[2]=1.0;

	float cos_teta=normalVec[0]/(sqrt(normalVec[0]*normalVec[0]+normalVec[1]*normalVec[1]+normalVec[2]*normalVec[2]));
	if (normalVec[1]>0.0)
		teta=acos(cos_teta);
	else
		teta=-1.0*acos(cos_teta);

	objFrame[0]=teta;	//yaw
	objFrame[1]=0.0;	//pitch
	objFrame[2]=0.0;	//roll
	objFrame[3]=trackedShape.x_pc_centroid;		//x
	objFrame[4]=trackedShape.y_pc_centroid;	 	//y
	objFrame[5]=trackedShape.z_pc_centroid-0.02;		//z

	class Frame tempCenter("CenterFrame",objFrame);
	objectFrames.push_back(tempCenter);

};

// =======================================

void pittObjects::Unknown::BoundingBox(void){
	cout<<"Unknown::BoundingBox"<<endl;


}

void pittObjects::Unknown::BoundingBall(void){
	cout<<"Unknown::BoundingBall"<<endl;

}

void pittObjects::Unknown::GraspingPosition(void){
	cout<<"Unknown::GraspingPosition"<<endl;

}
void pittObjects::Unknown::FrameSet(void){
	cout<<"Unknown::FrameSet"<<endl;

}
