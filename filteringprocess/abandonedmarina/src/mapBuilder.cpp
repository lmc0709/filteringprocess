//This program filters the map using algorithms in Probablist Robotics chapter 6
#include <ros/ros.h>
#include <ros/package.h>
  // PCL specific includes
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/common/transforms.h>
#include <pcl/common/geometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Transform.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <std_msgs/String.h>

// #include "map/map.h"
#include <mapClasses/map/map.h>
#include <mapClasses/scanBuffer/scanBuffer.h>
#include <mapclasses/buff.h>

#include <math.h> 
#include <vector>
#include <iostream>
#include <fstream>

#define inputSonarScan		"/scanFilterAB/buff"
#define inputMapToTXT		"/writeMapToTXT"
#define inputUpdateBuffer	"/mapBuilderAB/sonar_update"
#define outputMap			"map"
#define outputScan			"Detectedfeatures"
#define outputScanAligned	"DetectedFeaturesAligned"
#define outputScanUpdate	"sonar_update"
#define outputPillarUpdate	"pillar_update"
#define outputTrans			"trans_update"
#define outputLines			"lines_update"

typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

using namespace std;
using namespace Eigen;

class mapBuilder{
public:
    ros::NodeHandle nh;
private:
	ros::Subscriber sub;
	// ros::Subscriber subUpdate;
	ros::Subscriber subMapToTXT;
	ros::Publisher pubMap;
	ros::Publisher pubScan;
	ros::Publisher pubPillar;
	ros::Publisher pubScanAligned;
	ros::Publisher pubScanUpdate;
	// ros::Publisher pubTransform;
	// ros::Publisher pubLines;


	sonarMap map;
	scanBuffer sb;
	tf::TransformListener listener;

public:
	mapBuilder(){
		nh = ros::NodeHandle("~");
		sb.initialize(&nh);
		map.initialize(&nh);
		sub = nh.subscribe(inputSonarScan, 1, &mapBuilder::subscribeToCloud, this);
		// subMapToTXT = nh.subscribe(inputMapToTXT, 1, &mapBuilder::subscribeMapToTXT, this);
		// subUpdate = nh.subscribe(inputUpdateBuffer, 1, &mapBuilder::subscribeUpdateBuffer, this);
		pubMap = nh.advertise<PointCloudT>(outputMap, 1);
		pubScan = nh.advertise<PointCloudT>(outputScan, 1);
		pubPillar = nh.advertise<PointCloudT>(outputPillarUpdate, 1);
		pubScanAligned = nh.advertise<PointCloudT>(outputScanAligned, 1);
		pubScanUpdate = nh.advertise<std_msgs::Bool>(outputScanUpdate, 1);
		// pubTransform = nh.advertise<geometry_msgs::Transform>(outputTrans, 1);
		// pubLines = nh.advertise<visualization_msgs::Marker>(outputLines, 1);
		// sb.setPublisher(&pubLines);


		// For some stupid reason this is needed so that I don't get linker error for 
		// Extract indices and passThroughFilter and transformPointCloud.
		// string pathPackage = ros::package::getPath("mapclasses");
		pcl::ExtractIndices<PointT> extract;
		pcl::PassThrough<PointT> pass;
		PointCloudT::Ptr tmp (new PointCloudT());
		tmp->header.frame_id="odom";
		pcl_ros::transformPointCloud("/odom" , *tmp, *tmp, listener);
	}

	void loadFeatures(buff bu){
		// Check to see if the map currently has any features 
		// ******************************************************
		// TO-DO:
		// Look into using kd-tree instead of for loops to search nearest neighbours.
		// Probably not needed
		// ******************************************************
		PointCloudT::Ptr mapc (new PointCloudT());
		PointCloudT::Ptr aligned (new PointCloudT());
		PointCloudT::Ptr features (new PointCloudT());
		std::vector<int>  TdataAss ;
		std::vector<int>  erasedId ;
		int type;
		if(sb.newScan(bu, features, type )){
			if(map.newScan(features,TdataAss,erasedId, type)){ // alignment has been performed
				// pubMap.publish(map.returnMap());
				pubScan.publish(features);
				pubPillar.publish(map.returnPillars());
				std_msgs::Bool a;
				a.data = true;
				pubScanUpdate.publish(a);
				*mapc = *map.returnMap();
				for(size_t i = 0; i < mapc->points.size(); ++i){
					if(mapc->points.at(i).intensity < 2){
						mapc->points.at(i).intensity = 0.0;
					}
				}
				pubMap.publish(mapc);
				pubScanAligned.publish(map.returnAligned());
			}
		}
	}

	void subscribeToCloud(mapclasses::buff bu){
		buff butmp;
		butmp.dist = bu.dists;
		pcl::fromROSMsg(bu.cloud, *butmp.buffer);
		loadFeatures(butmp);
	}

	// void subscribeMapToTXT(std_msgs::String data){
	// 	ofstream myfile;
	// 	string path = "/home/unnar/catkin_ws/src/filteringprocess/filteringprocess/mapping/savedData/";
	// 	string type = ".txt";
	// 	string file = path + data.data + type;
	// 	myfile.open(file.c_str(), std::ofstream::out | std::ofstream::trunc);
	// 	std::vector<PointCloudT::Ptr> pillars = map.returnPillarPoints();
	// 	myfile << "PILLARS" << ".\n";
	// 	for(size_t i = 0; i < pillars.size(); ++i){
	// 		myfile << "Pillar: "<< i+1 << ".\n";
	// 		std::cout << "PILLAR: "<< i+1 << std::endl;
	// 		for(size_t j = 0; j < pillars[i]->points.size(); ++j){
	// 			myfile << pillars[i]->points[j].x << "," << pillars[i]->points[j].y << "\n"; 
	// 			std::cout << pillars[i]->points[j].x << "," << pillars[i]->points[j].y << std::endl; 
	// 		}	
	// 	}

	// 	std::vector<PointCloudT::Ptr> features = map.returnFeaturePoints();
	// 	myfile << "FEATURES" << ".\n";
	// 	for(size_t i = 0; i < features.size(); ++i){
	// 		myfile << "Feature: "<< i+1 << ".\n";
	// 		std::cout << "Feature: "<< i+1 << std::endl;
	// 		for(size_t j = 0; j < features[i]->points.size(); ++j){
	// 			myfile << features[i]->points[j].x << "," << features[i]->points[j].y << "\n"; 
	// 			std::cout << features[i]->points[j].x << "," << features[i]->points[j].y << std::endl; 
	// 		}	
	// 	}
	// 	myfile.close();
	// }

	// void subscribeUpdateBuffer(std_msgs::Bool tmp){
	// 	map.updateBuffer();
	// }

};

int main (int argc, char** argv)
{
	// Initialize ROS 
	ros::init (argc, argv, "mapBuilder");

	mapBuilder mb;
	// Spin
	ros::spin ();
	return(0);
}