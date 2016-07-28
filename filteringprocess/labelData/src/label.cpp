//This program filters the map using algorithms in Probablist Robotics chapter 6
#include <ros/ros.h>
  // PCL specific includes
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/common/transforms.h>
#include <pcl/common/geometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <stdio.h>
#include <dlib/svm_threaded.h>
#include <dlib/rand.h>
#include <dlib/svm.h>
#include <std_msgs/String.h>
#include <mapclasses/buff.h>

#include <iostream>
#include <fstream>

using namespace dlib;

typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

// declare the svm input type 
typedef matrix<double,40,1> sample_type;
typedef one_vs_one_trainer<any_trainer<sample_type> > ovo_trainer;
typedef linear_kernel<sample_type> kernel_type;

class labelData{
	ros::NodeHandle nh;
	ros::Subscriber sub;
	ros::Subscriber subAB;
	ros::Subscriber save;

	ros::Publisher pub;

	std::vector<sample_type> samples;
    std::vector<double> labels;

    ovo_trainer trainer;
    svm_c_linear_trainer<kernel_type> linear_trainer;

public:
	labelData(){
		nh    = ros::NodeHandle("~");
		subAB = nh.subscribe("/mapBuilderAB/buff", 0, &labelData::subscribeToCloudAB, this);
		sub   = nh.subscribe("/mapBuilder/buff", 0, &labelData::subscribeToCloud, this);
		save  = nh.subscribe("/saveTrainer", 0, &labelData::saveTrainer, this);
		pub   = nh.advertise<PointCloudT>("current_labeling", 1);
		linear_trainer.set_c(10);
		trainer.set_trainer(linear_trainer);
	}

	void subscribeToCloudAB(mapclasses::buff bu){
		labelData::classifyCloud(bu);
	}

	void subscribeToCloud(mapclasses::buff bu){
		labelData::classifyCloud(bu);
	}

	void classifyCloud(mapclasses::buff bu){	
		PointCloudT::Ptr cloud (new PointCloudT());
		pcl::fromROSMsg(bu.cloud, *cloud);

		pub.publish(cloud);

		int c;
		std::cout << " " << std::endl;
		c=getchar();
		std::cout << "#Points: " << cloud->points.size() << ", Enter the appropriate class: ";
		do {
			c=getchar();
			if(c != '\n'){
				c = c-48;
				if(c == 2){
					std::cout << "WALL" << std::endl;
					labels.push_back(2);
				} else if(c == 1){
					std::cout << "PILLAR" << std::endl;
					labels.push_back(3);
				} else {
					std::cout << "NOTHING" << std::endl;
					labels.push_back(1);
				}
				samples.push_back(labelData::createSampleType(bu));
			}
		} while (c != '\n');
	}

	sample_type createSampleType(mapclasses::buff bu){
		sample_type tmp;

		PointCloudT::Ptr cloud (new PointCloudT());
		pcl::fromROSMsg(bu.cloud, *cloud);

		if(cloud->points.size() > tmp.size()/2){
			float rem = (cloud->points.size()-(tmp.size()/2.0))/2.0;
			for(long i = 0; i < tmp.size()/2; ++i){
				// std::cout << "i: " << i << std::endl;
				tmp(i) = cloud->points[i+std::ceil(rem)].intensity;
				tmp(i+20) = bu.dists[i+std::ceil(rem)];
			}

		}
		else if(cloud->points.size() < tmp.size()/2){			
			float pad = (tmp.size()/2-cloud->points.size())/2.0;
			// std::cout << "i: " << i << std::endl;
			for(long i = 0; i < tmp.size(); ++i){
				tmp(i) = 0;
			}	

			for(long i = 0; i < cloud->points.size(); ++i){
				tmp(i+std::ceil(pad)) = (double)cloud->points[i].intensity;
				tmp(i+20+std::ceil(pad)) = bu.dists[i];
			}

		} else {
			for(long i = 0; i < cloud->points.size(); ++i){
				tmp(i) = cloud->points[i].intensity;
				tmp(i+20) = bu.dists[i];
			}
		}

		std::cout << "tmp: ";
		for(long i = 0; i < tmp.size(); ++i){
			std::cout << tmp(i) << " ";
		}
		std::cout << " " << std::endl;

		return tmp;
	}

	void saveTrainer(std_msgs::String string){
		
		std::ofstream myfile;
		string.data += ".txt";
		myfile.open(string.data.c_str(), std::ofstream::out | std::ofstream::trunc);
		for(size_t i = 0; i < samples.size(); ++i){
			for(size_t j = 0; j < samples[i].size(); ++j){
				myfile << samples[i](j) << ",";
			}
			myfile << labels[i] << "\n";	
		}
		myfile.close();

		vector_normalizer<sample_type> normalizer;
	    // Let the normalizer learn the mean and standard deviation of the samples.
	    normalizer.train(samples);
	    // now normalize each sample
	    for (unsigned long i = 0; i < samples.size(); ++i)
	        samples[i] = normalizer(samples[i]); 

	    
		std::cout << "cross validation accuracy: " << cross_validate_multiclass_trainer(trainer, samples, labels, 5) << std::endl;
	}

};

int main (int argc, char** argv)
{
	// Initialize ROS 
	ros::init (argc, argv, "labelData");

	labelData ld;
	// Spin
	ros::spin ();
	return(0);
}
