#include <iostream>
#include <ros/ros.h>
#include "pitt_msgs/TrackedShape.h"
#include "pitt_msgs/TrackedShapes.h"
#include "pitt_msgs/ClustersOutput.h"
#include "Objects.hpp"
#include <geometry_msgs/Vector3.h>
#include <knowledge_msgs/knowledgeSRV.h>
#include <boost/algorithm/string.hpp>
#include <std_msgs/Float64MultiArray.h>

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
float init_q_[2][7];
vector<Point> pointsVector;

int NumberSphere=1, NumberCylinder=1, NumberUnknown=0, NumberCone=0,NumberPlane=0;

bool obj_call_back_flag=true;
float perception_regionOperating[6];

// functions Def:
void CallBackJointValues_LeftArm(const std_msgs::Float64MultiArray& msg);
void CallBackJointValues_RightArm(const std_msgs::Float64MultiArray& msg);
void CallBackShapes(const TrackedShapes& outShapes);

bool IsPerceivedObjectInsideWS(string ObjectName, float *ObjectCenter);
bool KnowledgeQuery(knowledge_msgs::knowledgeSRV::Request &req, knowledge_msgs::knowledgeSRV::Response &res);
void readPointsVector(string pointsPath);

int main(int argc, char **argv)
{

	perception_regionOperating[0]=0.45;	perception_regionOperating[1]=-0.01;	perception_regionOperating[2]=0.045;	//! center
	perception_regionOperating[3]=0.99;	perception_regionOperating[4]=0.99;		perception_regionOperating[5]=0.60;		//! size


	ros::init(argc, argv, "knowledge");
	ros::NodeHandle nh;

	ros::Subscriber sub_shapes =nh.subscribe("ransac_segmentation/trackedShapes",10, CallBackShapes);
	ros::Subscriber sub_LeftQ  =nh.subscribe("Q_leftArm" ,10, CallBackJointValues_LeftArm);
	ros::Subscriber sub_RightQ =nh.subscribe("Q_rightArm",10, CallBackJointValues_RightArm);

	const char* home=getenv("HOME");
	string pointPath(home);
	pointPath=pointPath+"/catkin_ws/src/KNOWLEDGE/knowledge/files/points_TableAssembly.txt";

	readPointsVector(pointPath);


	ros::ServiceServer service = nh.advertiseService("knowledgeService",KnowledgeQuery);

	cout << "*****************" << endl;
	cout << "Knowledge Representation is alive: " << endl;

	ros::spin();
	return 1;
}


void CallBackJointValues_LeftArm(const std_msgs::Float64MultiArray& msg){
	for (int i=0;i<7;i++)
		init_q_[0][i]=msg.data[i];
};

void CallBackJointValues_RightArm(const std_msgs::Float64MultiArray& msg){
	for (int i=0;i<7;i++)
		init_q_[1][i]=msg.data[i];
};

//********************************************************************************
//********************************************************************************
void CallBackShapes(const TrackedShapes& outShapes){
	TrackedShape::Ptr outShape ( new TrackedShape);
	int obj_counter=0; //! No of objects that ransac recognize and is not unknown
	int cylinder_counter=1, sphere_counter=1, plane_counter=1, cone_counter=1, unknown_counter=1;
	string obj_name;

	//	NoCorrectRecognizedObject=0;
	if (obj_call_back_flag==true)
	{
		int	perceivedNoSphere=0, perceivedNoCylinder=0, perceivedNoPlane=0, perceivedNoCone=0, perceivedNoUnknown=0;
		objectsVector.clear();

		for (int i=0;i<outShapes.tracked_shapes.size();i++)
		{
			// if the recognized objects are in the working space add to object list otherwise do not add.
			float objectCenter[]={outShapes.tracked_shapes[i].x_pc_centroid, outShapes.tracked_shapes[i].y_pc_centroid,outShapes.tracked_shapes[i].z_pc_centroid};
			string objectName=outShapes.tracked_shapes[i].shape_tag;

			if(IsPerceivedObjectInsideWS(objectName,objectCenter))
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
					obj_name=objectName+to_string(plane_counter);

					objectsVector.emplace_back(make_shared <pittObjects::Plane>(objID,obj_name,tracked_Shape));
				}

				else if (objectName=="cylinder")
				{
					perceivedNoCylinder++;
					obj_name=objectName+to_string(cylinder_counter);

					objectsVector.emplace_back(make_shared <pittObjects::Cylinder>(objID,obj_name,tracked_Shape));
				}

				else if (objectName=="cone")
				{
					perceivedNoCone++;
					obj_name=objectName+to_string(cone_counter);
					objectsVector.emplace_back(make_shared <pittObjects::Cone>(objID,obj_name,tracked_Shape));
				}
				else if (objectName=="unknown")
				{
					perceivedNoUnknown++;
					obj_name=objectName+to_string(unknown_counter);
					objectsVector.emplace_back(make_shared <pittObjects::Unknown>(objID,obj_name,tracked_Shape));
				}
				else
				{ cout<<"The Object Tag is not defined in the knowledge base"<<objectName<<endl;}
			}
		}

		//		float vecBall6[6],vecBox6[6];
		//		if (!objectsVector.empty())
		//		{
		//			objectsVector.back()->BoundingBox(vecBox6);
		//			objectsVector.back()->BoundingBall(vecBall6);
		//			objectsVector.back()->FrameSet();
		//		}

		cout<<"objects vector size: "<<objectsVector.size()<<endl;
		for (int i=0;i<objectsVector.size();i++)
			objectsVector[i]->Print();

		if(perceivedNoCone==NumberCone && perceivedNoCylinder==NumberCylinder && perceivedNoPlane==NumberPlane && perceivedNoSphere==NumberSphere && perceivedNoUnknown==NumberUnknown)
		{
			for (int i=0;i<objectsVector.size();i++)
			{
				objectsVector[i]->FrameSet();
				objectsVector[i]->GraspingPosition();
			}

			cout<<BOLD("********** Object Vector *************")<<endl;
			for (int i=0;i<objectsVector.size();i++)
				objectsVector[i]->Print();

			obj_call_back_flag=false;
		}

	}

};

//***************************************************************************
//***************************************************************************
void readPointsVector(string pointsPath){
	ifstream file_path_ifStr(pointsPath.c_str());
	string line;
	vector<string> line_list;

	string delim_type=" ";
	pointsVector.empty();
	if (file_path_ifStr.is_open())
	{
		while(getline(file_path_ifStr,line))
		{
			boost::split(line_list, line, boost::is_any_of(delim_type));
			if(line_list[0]!="#")
			{
				vector<float> Pose;
				for(int i=1;i<line_list.size();i++)
				{
					Pose.push_back( stof(line_list[i]) );
				}

				Point temp_point(line_list[0],Pose);
				pointsVector.push_back(temp_point);
			}
		}
	}

	for(int i=0;i<pointsVector.size();i++)
		pointsVector[i].Print();
};

//***************************************************************************
//***************************************************************************

bool KnowledgeQuery(knowledge_msgs::knowledgeSRV::Request &req, knowledge_msgs::knowledgeSRV::Response &res){
	cout<<FBLU(BOLD("A Knowledge Query is arrived:"))<<endl;

	string type=req.reqType;
	string name=req.Name;
	string requestInfo=req.requestInfo;

	cout<<"name: "<<name<<", type: "<<type<<", requestInfo: "<<requestInfo<<endl;

	knowledge_msgs::Region region;
	geometry_msgs::Vector3 PoseLinear,PoseAngular;

	if(type.find("plane") != std::string::npos || type.find("cylinder") != std::string::npos || type.find("sphere") != std::string::npos)
	{
		if(requestInfo=="Pose")
		{
			for(int i=0;i<objectsVector.size();i++)
			{
				cout<< type<<":"<<objectsVector[i]->objName<<endl;
				if(type==objectsVector[i]->objName)
				{
					for(int j=0;j<objectsVector[i]->objectFrames.size();j++)
					{
						cout<< name<<":"<<objectsVector[i]->objectFrames[j].name<<endl;
						if(name==objectsVector[i]->objectFrames[j].name)
						{
							for(int k=0;k<6;k++)
							{
								res.pose.push_back(objectsVector[i]->objectFrames[j].frame[k]);
							}
							break;
						}

					}

					break;
				}
				else
				{
					cout<<"Object with name: "<<type<<" is not found in Knowledge base"<<endl;
				}
			}
		}
		else if(requestInfo=="boundingBox")
		{
			for(int i=0;i<objectsVector.size();i++)
			{
				float boundingBox[6];
				//				objectsVector[i]->BoundingBox(boundingBox);

				for(int j=0;j<6;j++)
				{
					region.data.push_back(boundingBox[j]);
				}
				res.region.push_back(region);
			}
		}
		else if(requestInfo=="boundingSphere")
		{
			for(int i=0;i<objectsVector.size();i++)
			{
				float boundingBall[4];
				//				objectsVector[i]->BoundingBall(boundingBall);

				for(int j=0;j<6;j++)
				{
					if(j<4)
						region.data.push_back(boundingBall[j]);
					else
						region.data.push_back(0.0);
				}
				res.region.push_back(region);
			}
		}
		else
		{
			cout<<"The request info is wrong: "<<requestInfo <<endl;
		}
	}
	else if(type.find("Point") != std::string::npos)
	{

		for(int i=0;i<pointsVector.size();i++)
		{
			if(type==pointsVector[i].name)
			{
				for(int j=0;j<pointsVector[i].pose.size();j++)
				{
					res.pose.push_back(pointsVector[i].pose[j]);
				}
				cout<<pointsVector[i].name<<" ";
				for(int j=0;j<pointsVector[i].pose.size();j++)
					cout<<pointsVector[i].pose[j]<<" ";
				cout<<endl;
				break;
			}

		}
	}
	else if(type.find("JointValues") != std::string::npos)
	{
		string delim_type="+";
		vector<string> agents_vector;
		boost::split(agents_vector, requestInfo, boost::is_any_of(delim_type));
		for(int i=0;i<agents_vector.size();i++)
		{
			region.data.clear();
			if(agents_vector[i]=="LeftArm")
				for(int j=0;j<7;j++)
					region.data.push_back(init_q_[0][j]);

			else if(agents_vector[i]=="RightArm")
				for(int j=0;j<7;j++)
					region.data.push_back(init_q_[1][j]);

			else
				cout<<"Error in incoming msg: "<<agents_vector[i]<<endl;

			res.region.push_back(region);
		}

		cout<<"left Arm q: ";
		for (int i=0;i<res.region[0].data.size();i++)
			cout<<res.region[0].data[i]<<" ";
		cout<<endl;
		cout<<"right Arm q: ";
		for (int i=0;i<res.region[1].data.size();i++)
			cout<<res.region[1].data[i]<<" ";
		cout<<endl;



	}
	else
	{
		cout<<"Request type is wrong:"<<type<<endl;
	}

	return true;
};

bool IsPerceivedObjectInsideWS(string ObjectName, float *ObjectCenter){
	bool isObjectInWS=true;
	float perception_regionOperating[6];
	perception_regionOperating[0]=0.45;	perception_regionOperating[1]=-0.01;	perception_regionOperating[2]=0.045;	//! center
	perception_regionOperating[3]=0.99;	perception_regionOperating[4]=0.99;		perception_regionOperating[5]=0.60;		//! size

	if( 	ObjectCenter[0]> (perception_regionOperating[0]+perception_regionOperating[3]/2.0) ||
			ObjectCenter[0]< (perception_regionOperating[0]-perception_regionOperating[3]/2.0) ||

			ObjectCenter[1]> (perception_regionOperating[1]+perception_regionOperating[4]/2.0) ||
			ObjectCenter[1]< (perception_regionOperating[1]-perception_regionOperating[4]/2.0) ||

			ObjectCenter[2]> (perception_regionOperating[2]+perception_regionOperating[5]/2.0) ||
			ObjectCenter[2]< (perception_regionOperating[2]-perception_regionOperating[5]/2.0) )
	{
		isObjectInWS=false;
		cout<<ObjectName<<" is Out of perception working space"<<endl;
		cout<<" X: object center:"<<ObjectCenter[0]<< ", Min WS: "<< perception_regionOperating[0]-perception_regionOperating[3]/2.0<<", Max WS: "<<perception_regionOperating[0]+perception_regionOperating[3]/2.0<<endl;
		cout<<" Y: object center:"<<ObjectCenter[1]<< ", Min WS: "<< perception_regionOperating[1]-perception_regionOperating[4]/2.0<<", Max WS: "<<perception_regionOperating[1]+perception_regionOperating[4]/2.0<<endl;
		cout<<" Z: object center:"<<ObjectCenter[2]<< ", Min WS: "<< perception_regionOperating[2]-perception_regionOperating[5]/2.0<<", Max WS: "<<perception_regionOperating[2]+perception_regionOperating[5]/2.0<<endl;

	}

	return isObjectInWS;
};

