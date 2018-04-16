#include <iostream>
#include <ros/ros.h>
#include "pitt_msgs/TrackedShape.h"
#include "pitt_msgs/TrackedShapes.h"
#include "pitt_msgs/ClustersOutput.h"
#include "Objects.hpp"
#include "Points.hpp"
#include "World.hpp"
#include <geometry_msgs/Vector3.h>
#include <knowledge_msgs/knowledgeSRV.h>
#include <boost/algorithm/string.hpp>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
#include "controlCommnad_msgs/controlGoalReachAck.h"


#define RST  "\x1B[0m"
#define KBLU  "\x1B[34m"
#define KRED  "\x1B[31m"
#define KGRN  "\x1B[32m"
#define FBLU(x) KBLU x RST
#define FRED(x) KRED x RST
#define FGRN(x) KGRN x RST
#define BOLD(x) "\x1B[1m" x RST

typedef ::pitt_msgs::TrackedShapes_<std::allocator<void> > TrackedShapes;
typedef ::pitt_msgs::TrackedShape_<std::allocator<void> > TrackedShape;

using namespace std;
using namespace pitt_msgs;

// global variables:
vector<shared_ptr<pittObjects::Objects>> objectsVector;

vector<World> worldVec;
int left_q_index,right_q_index;
int kb_update_arm , kb_update_counter=0;

int NumberSphere=0, NumberCylinder=4, NumberUnknown=0, NumberCone=0,NumberPlane=1;

bool obj_call_back_flag=true;
float perception_regionOperating[6], reduction_WS[6];

// functions Def:
void CallBackJointValues_LeftArm(const std_msgs::Float64MultiArray& msg);
void CallBackJointValues_RightArm(const std_msgs::Float64MultiArray& msg);
void CallBackUpdateKB(const std_msgs::String::ConstPtr& msg);

void CallBackShapes(const TrackedShapes& outShapes);

bool IsPerceivedObjectInsideWS(string ObjectName, float *ObjectCenter, float* WS);
bool KnowledgeQuery(knowledge_msgs::knowledgeSRV::Request &req, knowledge_msgs::knowledgeSRV::Response &res);

ros::Publisher	pub_KBAck, pub_pitt_runner;
int main(int argc, char **argv)
{

	perception_regionOperating[0]=0.45;	perception_regionOperating[1]=-0.01;	perception_regionOperating[2]=0.045;	//! center
	perception_regionOperating[3]=0.99;	perception_regionOperating[4]=0.99;		perception_regionOperating[5]=0.60;		//! size

	reduction_WS[0]=0.0;	reduction_WS[1]=0.0;	reduction_WS[2]=0.045;	//! center
	reduction_WS[3]=0.0;	reduction_WS[4]=0.0;	reduction_WS[5]=0.60;		//! size




	ros::init(argc, argv, "knowledge");
	ros::NodeHandle nh;

	ros::Subscriber sub_shapes =nh.subscribe("ransac_segmentation/trackedShapes",10, CallBackShapes);
	ros::Subscriber sub_LeftQ  =nh.subscribe("Q_leftArm" ,10, CallBackJointValues_LeftArm);
	ros::Subscriber sub_RightQ =nh.subscribe("Q_rightArm",10, CallBackJointValues_RightArm);
	ros::Subscriber sub_UpdateKB =nh.subscribe("robot_KB_command",10, CallBackUpdateKB);
	pub_KBAck=nh.advertise<controlCommnad_msgs::controlGoalReachAck>("robot_control_ack",80);
	pub_pitt_runner=nh.advertise<std_msgs::String>("PerceptionRunner",80);


	const char* home=getenv("HOME");
	string pointPath(home);
	pointPath=pointPath+"/catkin_ws/src/KNOWLEDGE/knowledge/files/points_TableAssembly.txt";
	cout<<FBLU("Time: ")<<to_string(ros::Time::now().toSec())<<endl;

	readPointsVector(pointPath, worldVec);
	left_q_index=worldVec.size();
	right_q_index=worldVec.size()+1;
	worldVec.resize(worldVec.size()+2);

	worldVec[left_q_index].name.push_back("LeftArm_q");
	worldVec[right_q_index].name.push_back("RightArm_q");
	worldVec[left_q_index].name.push_back("Pose");
	worldVec[right_q_index].name.push_back("Pose");


	cout<<"left_q_index: "<<left_q_index<<"right_q_index: "<<right_q_index<<endl;
	worldVec[left_q_index].Print();
	worldVec[right_q_index].Print();

	ros::ServiceServer service = nh.advertiseService("knowledgeService",KnowledgeQuery);

	cout<<BOLD("******* World Vector ******")<<endl;
	cout<<BOLD("***************************")<<endl;
	for(int i=0;i<worldVec.size();i++)
	{
		cout<<i<<endl;
		worldVec[i].Print();
	}

	usleep(0.5e6);
	std_msgs::String ackMsgStr;
	ackMsgStr.data="RUN_PITT";
	pub_pitt_runner.publish(ackMsgStr);
	usleep(0.5e6);
	cout<<FBLU("Time: ")<<to_string(ros::Time::now().toSec())<<endl;
	cout << "*****************" << endl;
	cout << "Knowledge Representation is alive: " << endl;
	cout<<FBLU("Time: ")<<to_string(ros::Time::now().toSec())<<endl;
	ros::spin();
	return 1;
}

//***************************************************************************
//***************************************************************************

bool KnowledgeQuery(knowledge_msgs::knowledgeSRV::Request &req, knowledge_msgs::knowledgeSRV::Response &res){
	cout<<FBLU(BOLD("A Knowledge Query is arrived:"))<<endl;
	cout<<FBLU("Time: ")<<to_string(ros::Time::now().toSec())<<endl;
	string type=req.reqType;
	string name=req.Name;
	string requestInfo=req.requestInfo;

	cout<<"MSG:: type: "<<type<<", requestInfo: "<<requestInfo<<endl;
	vector<string> typeVec,requestInfoVec;
	boost::split(typeVec, type, boost::is_any_of("-")); // Exmpl: Point-Point1, Point1, Cylinder-Cylinder1, Cylinder, Cylinder-Cylinder1-graspingPose1, Cylinder-Cylinder1-centerFrame
	boost::split(requestInfoVec, requestInfo, boost::is_any_of("-"));// Pose, Pose-Name, Center, Center-Name, boundingBox, boundingBall
	bool ret_name=false; // trturn name (if false: value)

	if(requestInfo.find("Name") != std::string::npos)
	{
		ret_name=true;
		cout<<"return name"<<endl;
	}
	else
	{
		cout<<"return values"<<endl;
	}

	for(int i=0;i<worldVec.size();i++)
	{
		int Occurence=0;
		for(int j=0; j<worldVec[i].name.size();j++)// cylinder cylinder1 graspingPose1, point4 Pose,
		{
			for(int k=0;k<typeVec.size();k++) // cylinder1 graspingPose1
			{
				//				cout<<"worldVec["<<i<<"].name["<<j<<"]: "<<worldVec[i].name[j]<<", typeVec["<<k<<"]: "<<typeVec[k]<<endl;
				if(worldVec[i].name[j]==typeVec[k])
				{
					Occurence++;
					//					cout<<Occurence<<endl;
				}
			}
			if(Occurence==typeVec.size() && worldVec[i].name.back().find(requestInfoVec[0]) != std::string::npos)
			{
				if(ret_name==true)
				{
					string KB_name;
					KB_name+=worldVec[i].name[0];
					for(int l=1; l<worldVec[i].name.size();l++)
						KB_name+="-"+worldVec[i].name[l];

					cout<<"Res: "<<KB_name<<endl;
					res.names.push_back(KB_name);
				}
				else
				{
					cout<<"Res: ";
					for(int m=0;m<worldVec[i].value.size();m++)
					{
						res.pose.push_back(worldVec[i].value[m]);
						cout<<worldVec[i].value[m]<<" ";
					}
					cout<<endl;
					cout<<FBLU("Time: ")<<to_string(ros::Time::now().toSec())<<endl;
					return true;// normally when ask for a vector value, it is just one vector value
				}
				break; // if a index of world vec happened, the same index should be passed and not happen again.
			}
		}
	}
	cout<<FBLU("Time: ")<<to_string(ros::Time::now().toSec())<<endl;
	return true;
};



void CallBackJointValues_LeftArm(const std_msgs::Float64MultiArray& msg){
	vector<float> leftQ;
	for (int i=0;i<7;i++)
		leftQ.push_back(msg.data[i]);
	worldVec[left_q_index].value=leftQ;

	//	worldVec[left_q_index].Print();

};

void CallBackJointValues_RightArm(const std_msgs::Float64MultiArray& msg){
	vector<float> rightQ;
	for (int i=0;i<7;i++)
		rightQ.push_back(msg.data[i]);

	worldVec[right_q_index].value=rightQ;

	//	worldVec[right_q_index].Print();

};

//********************************************************************************
//********************************************************************************
void CallBackShapes(const TrackedShapes& outShapes){
	//	cout<<FGRN(BOLD("CallBackShapes"))<<endl;

	TrackedShape::Ptr outShape ( new TrackedShape);
	string obj_name;

	//	NoCorrectRecognizedObject=0;
	if (obj_call_back_flag==true)
	{
		int	perceivedNoSphere=0, perceivedNoCylinder=0, perceivedNoPlane=0, perceivedNoCone=0, perceivedNoUnknown=0;
		objectsVector.clear();
		cout<<FBLU("Time: ")<<to_string(ros::Time::now().toSec())<<endl;

		for (int i=0;i<outShapes.tracked_shapes.size();i++)
		{
			// if the recognized objects are in the working space add to object list otherwise do not add.
			float objectCenter[]={outShapes.tracked_shapes[i].x_pc_centroid, outShapes.tracked_shapes[i].y_pc_centroid,outShapes.tracked_shapes[i].z_pc_centroid};
			string objectName=outShapes.tracked_shapes[i].shape_tag;

			if(IsPerceivedObjectInsideWS(objectName,objectCenter,perception_regionOperating) && !IsPerceivedObjectInsideWS(objectName,objectCenter,reduction_WS) )
			{
				int objID=outShapes.tracked_shapes[i].object_id;
				TrackedShape tracked_Shape=outShapes.tracked_shapes[i];

				if (objectName=="sphere")
				{
					perceivedNoSphere++;
					obj_name=objectName+to_string(perceivedNoSphere);

					objectsVector.emplace_back(make_shared <pittObjects::Sphere>(objID,obj_name,tracked_Shape));
				}

				else if (objectName=="plane")
				{
					perceivedNoPlane++;
					obj_name=objectName+to_string(perceivedNoPlane);

					objectsVector.emplace_back(make_shared <pittObjects::Plane>(objID,obj_name,tracked_Shape));
				}

				else if (objectName=="cylinder")
				{
					perceivedNoCylinder++;
					obj_name=objectName+to_string(perceivedNoCylinder);

					objectsVector.emplace_back(make_shared <pittObjects::Cylinder>(objID,obj_name,tracked_Shape));
				}

				else if (objectName=="cone")
				{
					perceivedNoCone++;
					obj_name=objectName+to_string(perceivedNoCone);
					objectsVector.emplace_back(make_shared <pittObjects::Cone>(objID,obj_name,tracked_Shape));
				}
				else if (objectName=="unknown")
				{
					perceivedNoUnknown++;
					obj_name=objectName+to_string(perceivedNoUnknown);
					objectsVector.emplace_back(make_shared <pittObjects::Unknown>(objID,obj_name,tracked_Shape));
				}
				else
				{ cout<<"The Object Tag is not defined in the knowledge base"<<objectName<<endl;}
			}
		}


		cout<<"objects vector size: "<<objectsVector.size()<<endl;
		for (int i=0;i<objectsVector.size();i++)
			objectsVector[i]->Print();
		cout<<"+++++++++++++++++++++++++++++++"<<endl;

		if((perceivedNoCone==NumberCone && perceivedNoCylinder==NumberCylinder && perceivedNoPlane==NumberPlane && perceivedNoSphere==NumberSphere && perceivedNoUnknown==NumberUnknown)||
				(kb_update_counter==0 && perceivedNoPlane==NumberPlane ))
		{
			for (int i=0;i<objectsVector.size();i++)
			{
				bool graspingPose;
				objectsVector[i]->FrameSet();
				graspingPose=objectsVector[i]->GraspingPosition();
				if(graspingPose==false) // if the rectangle vertices are not theoretically well found!
					return;
			}

			cout<<BOLD("********** Object Vector *************")<<endl;
			for (int i=0;i<objectsVector.size();i++)
			{
				objectsVector[i]->Print();
				for(int j=0;j<objectsVector[i]->objectFrames.size();j++)
				{
					World instance;
					instance.name.push_back(objectsVector[i]->objType);
					instance.name.push_back(objectsVector[i]->objName);
					for(int k=0;k<objectsVector[i]->objectFrames[j].name.size();k++)
						instance.name.push_back(objectsVector[i]->objectFrames[j].name[k]);

					for(int m=0;m<6;m++)
						instance.value.push_back(objectsVector[i]->objectFrames[j].frame[m]);
					worldVec.push_back(instance);
				}
			}
			cout<<BOLD("******* World Vector ******")<<endl;
			cout<<BOLD("***************************")<<endl;
			for(int i=0;i<worldVec.size();i++)
				worldVec[i].Print();

			controlCommnad_msgs::controlGoalReachAck ackMsg;
			ackMsg.armState=kb_update_arm;
			ackMsg.ctrlCmndTypeAck=0;
			pub_KBAck.publish(ackMsg);

			std_msgs::String ackMsgStr;
			ackMsgStr.data="KILL_PITT";
			pub_pitt_runner.publish(ackMsgStr);
			cout<<FBLU("Time: ")<<to_string(ros::Time::now().toSec())<<endl;
			obj_call_back_flag=false;


		}
	}

};


void CallBackUpdateKB(const std_msgs::String::ConstPtr& msg){ //Reduce_WS 1 Reduce_cylinder 0 ...
	cout<<FBLU("Time: ")<<to_string(ros::Time::now().toSec())<<endl;
	std_msgs::String ackMsgStr;
	ackMsgStr.data="RUN_PITT";
	pub_pitt_runner.publish(ackMsgStr);


	cout<<FRED(BOLD("CallBackUpdateKB"))<<endl;
	string Msg=msg->data.c_str();
	cout<<"arriving msg: "<<Msg<<endl;

	cout<<"NumberCone :"<<NumberCone <<endl;
	cout<<"NumberCylinder :"<<NumberCylinder<<endl;
	cout<<"NumberPlane :"<<NumberPlane <<endl;
	cout<<"NumberSphere :"<<NumberSphere <<endl;
	cout<<"NumberUnknown :"<<NumberUnknown <<endl;
	cout<<"WS: "<<reduction_WS[0]<<" "<<reduction_WS[1]<<" "<<reduction_WS[2]<<" "<<reduction_WS[3]<<" "<<reduction_WS[4]<<" "<<reduction_WS[5]<<endl;

	vector<string> msgvector, actionVector, actionParameter;
	boost::split(msgvector, Msg, boost::is_any_of(" "));
	kb_update_arm=stoi(msgvector[1]);

	//	boost::split(actionVector, msgvector[0], boost::is_any_of("_"));
	if(kb_update_counter==0)
	{

		for(vector<World>::iterator it=worldVec.begin(); it!=worldVec.end(); )
			if(it->name[0]=="cylinder" ||it->name[0]=="plane" )
				worldVec.erase(it);
			else
				it++;
	}
	else if(kb_update_counter==1)
	{
		//		boost::split(actionParameter, actionVector[1], boost::is_any_of("-"));
		//		if(actionParameter[0]=="WS")
		//		{
		NumberPlane--;
		NumberCylinder--;

		knowledge_msgs::knowledgeSRV::Request request;
		knowledge_msgs::knowledgeSRV::Response response;
		float planeVertx[3];
		float maxX=-10000.0,minX=10000.0, maxY=-100000.0, minY=100000.0;

		request.reqType="plane1-VertexPose1";
		request.requestInfo="Pose";
		KnowledgeQuery(request,response);

		planeVertx[0]= response.pose[3]; planeVertx[1]= response.pose[4]; planeVertx[2]= response.pose[5];
//		cout<<"1 planeVertx:"<<planeVertx[0]<<","<<planeVertx[1]<<","<<planeVertx[2]<<endl;
//		cout<<"1  max x"<<maxX<<", min x: "<<minX<<", max y: "<<maxY<<", min y: "<<minY<<endl;
		if(planeVertx[0]>maxX){ 	maxX=planeVertx[0]; }
		if(planeVertx[0]<minX){ 	minX=planeVertx[0]; }
		if(planeVertx[1]>maxY){ 	maxY=planeVertx[1]; }
		if(planeVertx[1]<minY){		minY=planeVertx[1]; }

		response.pose.clear();
		request.reqType="plane1-VertexPose2";
		request.requestInfo="Pose";
		KnowledgeQuery(request,response);
		planeVertx[0]= response.pose[3]; planeVertx[1]= response.pose[4]; planeVertx[2]= response.pose[5];

//		cout<<"2 planeVertx:"<<planeVertx[0]<<","<<planeVertx[1]<<","<<planeVertx[2]<<endl;
//		cout<<"2 max x"<<maxX<<", min x: "<<minX<<", max y: "<<maxY<<", min y: "<<minY<<endl;
		if(planeVertx[0]>maxX){ 	maxX=planeVertx[0]; }
		if(planeVertx[0]<minX){ 	minX=planeVertx[0]; }
		if(planeVertx[1]>maxY){ 	maxY=planeVertx[1]; }
		if(planeVertx[1]<minY){		minY=planeVertx[1]; }

		response.pose.clear();
		request.reqType="plane1-VertexPose3";
		request.requestInfo="Pose";
		KnowledgeQuery(request,response);
		planeVertx[0]= response.pose[3]; planeVertx[1]= response.pose[4]; planeVertx[2]= response.pose[5];
//		cout<<"3 planeVertx:"<<planeVertx[0]<<","<<planeVertx[1]<<","<<planeVertx[2]<<endl;
//		cout<<"3 max x"<<maxX<<", min x: "<<minX<<", max y: "<<maxY<<", min y: "<<minY<<endl;
		if(planeVertx[0]>maxX){ 	maxX=planeVertx[0]; }
		if(planeVertx[0]<minX){ 	minX=planeVertx[0]; }
		if(planeVertx[1]>maxY){ 	maxY=planeVertx[1]; }
		if(planeVertx[1]<minY){		minY=planeVertx[1]; }

		response.pose.clear();
		request.reqType="plane1-VertexPose4";
		request.requestInfo="Pose";
		KnowledgeQuery(request,response);
		planeVertx[0]= response.pose[3]; planeVertx[1]= response.pose[4]; planeVertx[2]= response.pose[5];
//		cout<<"4 planeVertx:"<<planeVertx[0]<<","<<planeVertx[1]<<","<<planeVertx[2]<<endl;
//		cout<<"4 max x"<<maxX<<", min x: "<<minX<<", max y: "<<maxY<<", min y: "<<minY<<endl;
		if(planeVertx[0]>maxX){ 	maxX=planeVertx[0]; }
		if(planeVertx[0]<minX){ 	minX=planeVertx[0]; }
		if(planeVertx[1]>maxY){ 	maxY=planeVertx[1]; }
		if(planeVertx[1]<minY){		minY=planeVertx[1]; }

		reduction_WS[0]= (maxX+minX)/2.0;
		reduction_WS[1]= (maxY+minY)/2.0;
		reduction_WS[3]=maxX-minX;
		reduction_WS[4]=maxY-minY;
		cout<<"max x: "<<maxX<<", min x: "<<minX<<", max y: "<<maxY<<", min y: "<<minY<<endl;
		cout<<"Reduced ws: "<<reduction_WS[0]<<" "<<reduction_WS[1]<<" "<<reduction_WS[2]<<", "<<reduction_WS[3]<<" "<<reduction_WS[4]<<" "<<reduction_WS[5]<<endl;
	}
	//		else if(actionParameter[0]=="cylinder")
	else
	{
		NumberCylinder--;
	}

	for(vector<World>::iterator it=worldVec.begin(); it!=worldVec.end(); )
		if(it->name[0]=="cylinder")
			worldVec.erase(it);
		else
			it++;

	obj_call_back_flag=true;
//}
//	else
//	{
//		cout<<"Error in arriving msg: "<<msgvector[0]<<endl;
//	}



cout<<"NumberCone :"<<NumberCone <<endl;
cout<<"NumberCylinder :"<<NumberCylinder<<endl;
cout<<"NumberPlane :"<<NumberPlane <<endl;
cout<<"NumberSphere :"<<NumberSphere <<endl;
cout<<"NumberUnknown :"<<NumberUnknown <<endl;
cout<<"WS: "<<reduction_WS[0]<<" "<<reduction_WS[1]<<" "<<reduction_WS[2]<<" "<<reduction_WS[3]<<" "<<reduction_WS[4]<<" "<<reduction_WS[5]<<endl;

kb_update_counter++;

};



bool IsPerceivedObjectInsideWS(string ObjectName, float *ObjectCenter, float* WS){
	bool isObjectInWS=true;
	//	float WS[6];
	//	WS[0]=0.45;	WS[1]=-0.01;	WS[2]=0.045;	//! center
	//	WS[3]=0.99;	WS[4]=0.99;		WS[5]=0.60;		//! size


	if( 	ObjectCenter[0]> (WS[0]+WS[3]/2.0) ||	ObjectCenter[0]< (WS[0]-WS[3]/2.0) ||

			ObjectCenter[1]> (WS[1]+WS[4]/2.0) ||	ObjectCenter[1]< (WS[1]-WS[4]/2.0) ||

			ObjectCenter[2]> (WS[2]+WS[5]/2.0) ||	ObjectCenter[2]< (WS[2]-WS[5]/2.0) )
	{
		isObjectInWS=false;
		cout<<ObjectName<<" is Out of perception working space"<<endl;
		cout<<" X: object center:"<<ObjectCenter[0]<< ", Min WS: "<< WS[0]-WS[3]/2.0<<", Max WS: "<<WS[0]+WS[3]/2.0<<endl;
		cout<<" Y: object center:"<<ObjectCenter[1]<< ", Min WS: "<< WS[1]-WS[4]/2.0<<", Max WS: "<<WS[1]+WS[4]/2.0<<endl;
		cout<<" Z: object center:"<<ObjectCenter[2]<< ", Min WS: "<< WS[2]-WS[5]/2.0<<", Max WS: "<<WS[2]+WS[5]/2.0<<endl;

	}

	return isObjectInWS;
};

