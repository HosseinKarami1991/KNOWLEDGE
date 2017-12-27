#include <iostream>
#include <ros/ros.h>
#include "pitt_msgs/TrackedShape.h"
#include "pitt_msgs/TrackedShapes.h"
#include "pitt_msgs/ClustersOutput.h"
#include "Objects.hpp"
#include <geometry_msgs/Vector3.h>
#include <knowledge_msgs/knowledgeSRV.h>
#include <boost/algorithm/string.hpp>

typedef ::pitt_msgs::TrackedShapes_<std::allocator<void> > TrackedShapes;
typedef ::pitt_msgs::TrackedShape_<std::allocator<void> > TrackedShape;

using namespace std;
using namespace pitt_msgs;

// global variables:
vector<shared_ptr<pittObjects::Objects>> objectsVector;
vector<Point> pointsVector;

int NumberSphere=0, NumberCylinder=0, NumberUnknown=0, NumberCone=0,NumberPlane=0;

bool obj_call_back_flag=true;
float perception_regionOperating[6];

// functions Def:
void CallBackShapes(const TrackedShapes& outShapes);

bool KnowledgeQuery(knowledge_msgs::knowledgeSRV::Request &req, knowledge_msgs::knowledgeSRV::Response &res);
void readPointsVector(string pointsPath);

int main(int argc, char **argv)
{

	perception_regionOperating[0]=0.45;	perception_regionOperating[1]=-0.01;	perception_regionOperating[2]=0.045;	//! center
	perception_regionOperating[3]=0.99;	perception_regionOperating[4]=0.99;		perception_regionOperating[5]=0.60;		//! size


	ros::init(argc, argv, "knowledge");
	ros::NodeHandle nh;

	ros::Subscriber sub_shapes =nh.subscribe("ransac_segmentation/trackedShapes",10, CallBackShapes);
	const char* home=getenv("HOME");
	string pointPath(home);
	pointPath=pointPath+"/catkin_ws/src/KNOWLEDGE/knowledge/files/points.txt";

	readPointsVector(pointPath);


	ros::ServiceServer service = nh.advertiseService("knowledgeService",KnowledgeQuery);

	cout << "*****************" << endl;
	cout << "Knowledge Representation is alive: " << endl;

	ros::spin();
	return 1;
}


void CallBackShapes(const TrackedShapes& outShapes){
	TrackedShape::Ptr outShape ( new TrackedShape);
	int obj_counter=0; //! No of objects that ransac recognize and is not unknown
	int cylinder_counter=1, sphere_counter=1, plane_counter=1, cone_counter=1, unknown_counter=1;
	string obj_name;

//	NoCorrectRecognizedObject=0;
	if (obj_call_back_flag==true){
	int	EstimatedNoSphere=0, EstimatedNoCylinder=0, EstimatedNoPlane=0, EstimatedNoCone=0, EstimatedNoUnknown=0;

	objectsVector.clear();
		bool flag_isObjectInWS=true;


		for (int i=0;i<outShapes.tracked_shapes.size();i++)
		{
			// if the recognized objects are in the working space add to object list otherwise do not add.
			flag_isObjectInWS=true;
			if(	outShapes.tracked_shapes[i].x_pc_centroid> (perception_regionOperating[0]+perception_regionOperating[3]/2.0) ||
				outShapes.tracked_shapes[i].x_pc_centroid< (perception_regionOperating[0]-perception_regionOperating[3]/2.0) ||

				outShapes.tracked_shapes[i].y_pc_centroid> (perception_regionOperating[1]+perception_regionOperating[4]/2.0) ||
				outShapes.tracked_shapes[i].y_pc_centroid< (perception_regionOperating[1]-perception_regionOperating[4]/2.0) ||

				outShapes.tracked_shapes[i].z_pc_centroid> (perception_regionOperating[2]+perception_regionOperating[5]/2.0) ||
				outShapes.tracked_shapes[i].z_pc_centroid< (perception_regionOperating[2]-perception_regionOperating[5]/2.0) )
			{
				flag_isObjectInWS=false;
			}

			cout<<"************** 1: "<<outShapes.tracked_shapes[i].shape_tag<<endl;
			cout<<"************** 1: "<<outShapes.tracked_shapes[i].x_pc_centroid<< " "<< perception_regionOperating[0]-perception_regionOperating[3]/2.0<<" "<<perception_regionOperating[0]+perception_regionOperating[3]/2.0<<endl;
			cout<<"************** 1: "<<outShapes.tracked_shapes[i].y_pc_centroid<< " "<< perception_regionOperating[1]-perception_regionOperating[4]/2.0<<" "<<perception_regionOperating[1]+perception_regionOperating[4]/2.0<<endl;
			cout<<"************** 1: "<<outShapes.tracked_shapes[i].z_pc_centroid<< " "<< perception_regionOperating[2]-perception_regionOperating[5]/2.0<<" "<<perception_regionOperating[2]+perception_regionOperating[5]/2.0<<endl;
			if(flag_isObjectInWS==true)
			{
			cout<<"************** 2: "<<outShapes.tracked_shapes[i].shape_tag<<endl;

				if (outShapes.tracked_shapes[i].shape_tag=="sphere"){
					cout<<"************** 3: "<<outShapes.tracked_shapes[i].shape_tag<<endl;
					int objID=outShapes.tracked_shapes[i].object_id;
					TrackedShape tracked_Shape=outShapes.tracked_shapes[i];

					obj_name="sphere"+	to_string(sphere_counter);
					sphere_counter++;
					objectsVector.emplace_back(make_shared <pittObjects::Sphere>(objID,obj_name,tracked_Shape));
//					EstimatedNoSphere++;
					//			objectsVector2.push_back(new pittObjects::Sphere(objID,tracked_Shape)); // as am example keep here this one.
					float vecBall6[6],vecBox6[6];
					if (!objectsVector.empty())
					{
						objectsVector.back()->BoundingBox(vecBox6);
						objectsVector.back()->BoundingBall(vecBall6);
						objectsVector.back()->FrameSet();
					}

					EstimatedNoSphere++;
					//
					//			boundBoxSphere(outShapes.tracked_shapes[i],obj_counter);
					//			boundBallSphere(outShapes.tracked_shapes[i],obj_counter);
					//			goalCenterSet(outShapes.tracked_shapes[i],obj_counter);
					//			NoCorrectRecognizedObject++;
				}
				//		if (outShapes.tracked_shapes[i].shape_tag=="cylinder")
				//			boundBoxCylinder(outShapes.tracked_shapes[i],obj_counter);

				if (outShapes.tracked_shapes[i].shape_tag=="plane")
				{
					int objID=outShapes.tracked_shapes[i].object_id;
					TrackedShape tracked_Shape=outShapes.tracked_shapes[i];

					obj_name="plane"+	to_string(plane_counter);
					plane_counter++;

					objectsVector.emplace_back(make_shared <pittObjects::Plane>(objID,obj_name,tracked_Shape));
					EstimatedNoPlane++;
//					EstimatedNoPlane++;
					//			planeFrameSet(outShapes.tracked_shapes[i],100);
					//			NoCorrectRecognizedObject++;
				}

				if (outShapes.tracked_shapes[i].shape_tag=="cylinder")
				{
					int objID=outShapes.tracked_shapes[i].object_id;
					TrackedShape tracked_Shape=outShapes.tracked_shapes[i];
					obj_name="cylinder"+	to_string(cylinder_counter);
					cylinder_counter++;


					objectsVector.emplace_back(make_shared <pittObjects::Cylinder>(objID,obj_name,tracked_Shape));
					EstimatedNoCylinder++;
				}

				if (outShapes.tracked_shapes[i].shape_tag=="cone")
				{
					int objID=outShapes.tracked_shapes[i].object_id;
					TrackedShape tracked_Shape=outShapes.tracked_shapes[i];
					obj_name="cone"+	to_string(cone_counter);
					cone_counter++;


					objectsVector.emplace_back(make_shared <pittObjects::Cone>(objID,obj_name,tracked_Shape));
					EstimatedNoCone++;
				}

				if (outShapes.tracked_shapes[i].shape_tag=="unknown")
				{
					int objID=outShapes.tracked_shapes[i].object_id;
					TrackedShape tracked_Shape=outShapes.tracked_shapes[i];
					obj_name="unknown"+	to_string(unknown_counter);
					unknown_counter++;

					objectsVector.emplace_back(make_shared <pittObjects::Unknown>(objID,obj_name,tracked_Shape));
					EstimatedNoUnknown++;
				}

//				if (outShapes.tracked_shapes[i].shape_tag=="sphere")
//					obj_counter++;
			}
		}
//		if (NoSphere==EstimatedNoSphere && NoPlane==EstimatedNoPlane && NoCylinder==EstimatedNoCylinder
//				&& NoCone==EstimatedNoCone && NoUnknown==EstimatedNoUnknown)
//		{
//			cout<<"*** Scene Recognition Is Correct ***"<<endl;
//			cout<<"objectsVector.size: "<<objectsVector.size()<<endl;
//			NoObstacles=objectsVector.size();

			//		cout<<"Normal Vector"<<endl;
			//		for (int i=0;i<objectsVector2.size();i++){
			//			cout<<objectsVector2[i]<<endl;
			//			objectsVector2[i]->Print();
			//		}
			//		cout<<"=================================="<<"Shared_ptr Vector"<<endl;
			float graspPose[6],pathPlanningPose[6];

			cout<<"objects vector size: "<<objectsVector.size()<<endl;
			for (int i=0;i<objectsVector.size();i++){
				cout<<"***********"<<objectsVector[i]<<"************"<<endl;
				objectsVector[i]->Print();
//				objectsVector[i]->FrameSet();
//				objectsVector[i]->GraspingPosition(graspPose,pathPlanningPose,"top");

			}

			if(EstimatedNoCone==NumberCone && EstimatedNoCylinder==NumberCylinder && EstimatedNoPlane==NumberPlane &&
					EstimatedNoSphere==NumberSphere && EstimatedNoUnknown==NumberUnknown)
			{
				obj_call_back_flag=false;
			}

//		}
//		else{
//			cout<<">>>>>> Scene Recognition Is 'NOT' Correct <<<<<<"<<endl;
//			cout<<"Normal Vector"<<endl;
//			//		cout<<"objectsVector2.size(): "<<objectsVector2.size()<<endl;
//			//		for(vector<pittObjects::Objects *>::iterator it = objectsVector2.begin(); it != objectsVector2.end(); ++it) {
//			//		  delete (*it);
//			//		}
//			//		objectsVector2.clear();
//			//		cout<<"objectsVector2.size(): "<<objectsVector2.size()<<endl;
//			//
//			//		cout<<"=================================="<<"Shared_ptr Vector"<<endl;
//			cout<<"objectsVector2.size(): "<<objectsVector.size()<<endl;
//			objectsVector.clear();
//			cout<<"objectsVector.size(): "<<objectsVector.size()<<endl;
//		}

	}
	if(		obj_call_back_flag==false)
	{
		cout<<"========================================================="<<endl;

		for(int i=0;i<objectsVector.size();i++)
			objectsVector[i]->Print();
	}
};
//***************************************************************************
//***************************************************************************
void readPointsVector(string pointsPath){
	cout<<200<<endl;
	ifstream file_path_ifStr(pointsPath.c_str());
	string line;
	vector<string> line_list;

	string delim_type=" ";
	pointsVector.empty();
	cout<<201<<endl;
	if (file_path_ifStr.is_open())
	{
		while(getline(file_path_ifStr,line))
		{

			boost::split(line_list, line, boost::is_any_of(delim_type));
			//			Full_State_action_list.push_back(line_list);
			//			temp_point.name=line_list[0];
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
	cout<<"A Knowledge Query is arrived:"<<endl;

	string name=req.Name;
	string type=req.reqType;
	string requestInfo=req.requestInfo;

	cout<<"name: "<<name<<", type: "<<type<<", requestInfo: "<<requestInfo<<endl;

	knowledge_msgs::Region region;
	geometry_msgs::Vector3 PoseLinear,PoseAngular;

	if(type=="Object")
	{
		if(requestInfo=="graspPose")
		{
			for(int i=0;i<objectsVector.size();i++)
			{
				if(name==objectsVector[i]->objType)
				{
					for(int j=0;j<6;j++)
					{
						res.pose.push_back(objectsVector[i]->objectGraspPose[j]);
					}
				}

			}
		}
		else if(requestInfo=="boundingBox")
		{
			for(int i=0;i<objectsVector.size();i++)
			{
				float boundingBox[6];
				objectsVector[i]->BoundingBox(boundingBox);

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
				objectsVector[i]->BoundingBall(boundingBall);

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
	else if(type=="Point")
	{

		for(int i=0;i<pointsVector.size();i++)
		{
			if(name==pointsVector[i].name)
			{
				for(int j=0;j<pointsVector[i].pose.size();j++)
				{
					res.pose.push_back(pointsVector[i].pose[j]);
				}
			}

		}
	}
	else{
		cout<<"Request type is wrong:"<<type<<endl;
	}



return true;
};

