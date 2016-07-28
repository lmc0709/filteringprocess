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
#include <sstream>

#include <iostream>
#include <fstream>

using namespace dlib;

typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

// declare the svm input type 
typedef matrix<double,20,1> sample_type_short;
typedef matrix<double,40,1> sample_type;
typedef one_vs_one_trainer<any_trainer<sample_type> > ovo_trainer;
typedef linear_kernel<sample_type> kernel_type;
typedef radial_basis_kernel<sample_type> rbf_kernel;
typedef polynomial_kernel<sample_type> poly_kernel;
typedef decision_function<poly_kernel> dec_funct_type;
typedef probabilistic_decision_function<poly_kernel> prob_dec_funct_type;
typedef normalized_function<dec_funct_type> funct_type;
// typedef normalized_function<prob_dec_funct_type> prob_funct_type;

class trainSVM{
	ros::NodeHandle nh;
	ros::Subscriber sub;
	ros::Subscriber subAB;
	ros::Subscriber save;

	ros::Publisher pub;

	std::vector<sample_type> samples;
	std::vector<sample_type> samples_train;
	std::vector<sample_type> samples_test;
	std::vector<sample_type_short> samples_short;
	std::vector<std::vector<double> > dists;
    std::vector<double> labels;
    std::vector<double> labels_train;
    std::vector<double> labels_test;

    ovo_trainer trainer;
    ovo_trainer trainer2;
    ovo_trainer trainer3;
    svm_c_linear_trainer<kernel_type> linear_trainer;
    krr_trainer<rbf_kernel> rbf_trainer;
    svm_nu_trainer<poly_kernel> poly_trainer;

public:
	trainSVM(){
		nh    = ros::NodeHandle("~");
		// subAB = nh.subscribe("/mapBuilderAB/buff", 0, &labelData::subscribeToCloudAB, this);
		// sub   = nh.subscribe("/mapBuilder/buff", 0, &labelData::subscribeToCloud, this);
		// save  = nh.subscribe("/saveTrainer", 0, &labelData::saveTrainer, this);
		// pub   = nh.advertise<PointCloudT>("current_labeling", 1);
		rbf_trainer.set_kernel(rbf_kernel(0.1));
		linear_trainer.set_c(10);
		poly_trainer.set_kernel(poly_kernel(0.1, 1, 2));
		trainer.set_trainer(poly_trainer);
	}


	void doStuff(void){

	    trainSVM::readFile("pillar",samples_short, dists, labels);
	    trainSVM::readFile("abandoned",samples_short, dists, labels);
	    	
    	std::cout << "hehe" << std::endl;
	    sample_type_short tmpsample = samples_short[0]; 

	    std::cout << "haha" << std::endl;
	    vector_normalizer<sample_type_short> normalizer;
	    // Let the normalizer learn the mean and standard deviation of the samples.
	    normalizer.train(samples_short);
	    std::cout << "mean: " << normalizer.means() << std::endl;
	    // now normalize each sample
	    double mean = 0.0;
	    double firsttmp = 0;
	    for (unsigned long i = 0; i < samples_short.size(); ++i){
	    	firsttmp = samples_short[i](0);
	    	// normalizer.train(samples_short[i]);
	        samples_short[i] = normalizer(samples_short[i]);
	        mean = 0.0;
	        for(int k = 0; k < samples_short[i].size(); ++k)
	        {
	        	mean += samples_short[i](k);
	        }
	        // std::cout << "mean: " << mean/samples_short[i].size() << ", " << firsttmp-samples_short[i](0) << std::endl;
	    }
	    double result;
	    for(int i = 0; i < dists.size(); ++i)
	    {
    		result = *std::max_element(dists[i].begin(), dists[i].end());
    		for(int j = 0; j < dists[i].size(); ++j)
    		{
    			dists[i][j] /= result;
    		}
	    }
	    samples.resize(samples_short.size());
	    for(int i = 0; i < samples.size(); ++i){
	    	for(int j = 0; j < samples_short[i].size(); ++j){
	    		samples[i](j) = samples_short[i](j);
	    	}
	    	for(int j = 0; j < dists[i].size(); ++j){
	    		samples[i](j+20) = dists[i][j];
	    	}
	    }

	    // for(int i = 0; i < 10; ++i)
	    // {
	    // 	for(int j = 0; j < samples[i].size(); ++j)
	    // 	{
	    // 		std::cout << " " << samples[i](j);
	    		
	    // 	}
	    // 	std::cout << " " << std::endl;
	    // }
	    

	    // Linear trainer: C = 443
	    // radial basis trainer: kernel = 0.05

	    randomize_samples(samples, labels);
	    matrix<double> matd;
	    double accuracy = 0;
	    double accuracyHigh = 0;
	    int accuracyIdx = 0;
	    for(int i = 1; i < 2; ++i)
	    {
	    	std::cout << "hoho" << std::endl;
		    // linear_trainer.set_c(i);
		    // rbf_trainer.set_kernel(rbf_kernel(i/100.0));
		    poly_trainer.set_kernel(poly_kernel(0.6, 1, 2));
			trainer.set_trainer(poly_trainer);
			matd = cross_validate_multiclass_trainer(trainer, samples, labels, 5);
			std::cout << matd << std::endl;
			accuracy = (matd(0,0)/(matd(0,0)+matd(0,1)+matd(0,2)) + matd(1,1)/(matd(1,0)+matd(1,1)+matd(1,2)) + matd(2,2)/(matd(2,0)+matd(2,1)+matd(2,2)))/3.0;
			std::cout << "accuracy: " << accuracy << std::endl;
			std::cout << " " << std::endl;
			if(accuracy>accuracyHigh){
				accuracyHigh = accuracy;
				accuracyIdx = i;
			}
	    }
	    std::cout << "cross validation accuracy, C value: " << accuracyIdx << std::endl; 
	    // linear_trainer.set_c(accuracyIdx);
	    // rbf_trainer.set_kernel(rbf_kernel(accuracyIdx/100.0));
		poly_trainer.set_kernel(poly_kernel(0.6, 1, 2));
		trainer.set_trainer(poly_trainer);
	    std::cout << cross_validate_multiclass_trainer(trainer, samples, labels, 5) << std::endl;
		std::cout << "accuracy: " << accuracyHigh << std::endl;


		////////////////////////////////////////////////////////////////////////////////////////

		for(int i = 0; i < samples.size(); i++){
			if(i < samples.size()*0.5){
				samples_train.push_back(samples[i]);
				labels_train.push_back(labels[i]);
			} else {
				samples_test.push_back(samples[i]);
				labels_test.push_back(labels[i]);
			}
		}

		one_vs_one_decision_function<ovo_trainer> df = trainer.train(samples_train, labels_train);
		one_vs_one_decision_function<ovo_trainer, decision_function<poly_kernel> > df2, df3;

		df2 = df;
		// df2.normalizer = normalizer;
		serialize("df.dat") << df2;
		serialize("norm.dat") << normalizer;
        // load the function back in from disk and store it in df3.  
        deserialize("df.dat") >> df3;
        vector_normalizer<sample_type_short> normalizer2;
        deserialize("norm.dat") >> normalizer2;

        std::cout << "test deserialized function: \n" << test_multiclass_decision_function(df3, samples_test, labels_test) << std::endl;



	    rbf_trainer.set_kernel(rbf_kernel(0.05));
        trainer2.set_trainer(rbf_trainer);
		one_vs_one_decision_function<ovo_trainer> dfrbf = trainer2.train(samples_train, labels_train);
        std::cout << "test deserialized function rbf: \n" << test_multiclass_decision_function(dfrbf, samples_test, labels_test) << std::endl;


        linear_trainer.set_c(443);
        trainer3.set_trainer(linear_trainer);
        one_vs_one_decision_function<ovo_trainer> dflin = trainer3.train(samples_train, labels_train);
        std::cout << "test deserialized function linear: \n" << test_multiclass_decision_function(dflin, samples_test, labels_test) << std::endl;


		////////////////////////////////////////////////////////////////////////////////////////

        // std::cout << " " << std::endl;
        // std::cout << "pillar 1: " << df3(samples[0]) << ",  real: " << labels[0] << std::endl;
        // samples[0] = df3.normalizer(samples[0]);
                
        int correct = 0;
        int wrong = 0;
        for(int i = 0; i < samples.size(); ++i)
        {
        	if(df3(samples[i]) == labels[i]){
        		correct++;
        	} else {
        		wrong++;
        	}
        }
        std::cout << "percentage: " << float(correct)/(float)(correct+wrong) << std::endl;


        // Finally, if you want to get the binary classifiers from inside a multiclass decision
        // function you can do it by calling get_binary_decision_functions() like so:
        one_vs_one_decision_function<ovo_trainer>::binary_function_table functs;
        functs = df3.get_binary_decision_functions();
        std::cout << "number of binary decision functions in df: " << functs.size() << std::endl;
        // The functs object is a std::map which maps pairs of labels to binary decision
        // functions.  So we can access the individual decision functions like so:
        decision_function<poly_kernel> df_1_2 = any_cast<decision_function<poly_kernel> >(functs[make_unordered_pair(1,3)]);
        // decision_function<rbf_kernel>  df_1_3 = any_cast<decision_function<rbf_kernel>  >(functs[make_unordered_pair(1,3)]);
        // df_1_2 contains the binary decision function that votes for class 1 vs. 2.
        probabilistic_decision_function<poly_kernel> dfp1_2(1.0,0.0,df_1_2);

        // for(int k = 0; k < 100; ++k)
        // {
        // 	if(labels[k] != 2){
	       //  	std::cout << df3(samples[k]) << "  " << labels[k] << "    " << dfp1_2(samples[k]) << std::endl;
	       //  	/* code */
	       //  }
        // }
        serialize("df_1_3.dat") << dfp1_2;
        
	}


	void readFile(std::string str, std::vector<sample_type_short> &intensities, std::vector<std::vector<double> > &dists, std::vector<double> &label){
		// std::cout << "hahaha" << std::endl;
		std::string line;
		int i = 0;
		str += ".txt";
		std::ifstream myfile(str.c_str());
		sample_type_short sth;
		std::vector<double> dist(20);
		if (myfile.is_open())
		{
			// std::cout << "hihihih" << std::endl;
			while ( getline (myfile,line) )
			{	
				// std::cout << "hehehe" << std::endl;
				std::stringstream ss(line);   
			    float n;
			    i = 0;
			    while (ss >> n) {
			        
			        // std::cout << " " << n;
		         	
		         	if(i < 20){
		         		sth(i) = n;
		         	} else if(i < 40){
		         		// std::cout << "i-20" << std::endl;
		         		dist[i-20] = n;
		         	}
		         	else{
		         		label.push_back(n);
		         	}

		         	if (ss.peek() == ','){
        				ss.ignore();
		         	}
			        // std::cout << "hoho" << std::endl;
			        i++;
			    } 
			    // std::cout << "huhu" << std::endl;
			    intensities.push_back(sth);
			    dists.push_back(dist);
			    // std::cout << " " << std::endl;
			    if(i > 41){
			    	break;
			    }
			}
			myfile.close();
		}
		std::cout << "intensities: " << intensities.size() << std::endl;
		std::cout << "dists: " << dists.size() << std::endl;
		std::cout << "labels: " << labels.size() << std::endl;
	}

};

int main (int argc, char** argv)
{
	// Initialize ROS 
	ros::init (argc, argv, "trainSVM");

	trainSVM tsvm;
	tsvm.doStuff();
	// Spin
	// ros::spin ();
	return(0);
}
