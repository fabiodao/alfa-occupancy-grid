#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include <string>
#include <cmath>
#include <math.h>
#include "LinkedList.hpp"
#include <fstream>
#include <sstream>
#include <vector>
#include <cstring>
#include <pcl/registration/icp.h>


#define DUMMY_ID 0
#define ICP
#define PARAMETER_MULTIPLIER 1000

#define delta_ang 0.145

#define num_sections 43
#define num_bins_per_section 4

int m_bin = 0;
int n_section = 0;
double pi = 3.14159;
float angle = 0;
float tp = 0;
float fp = 0;
float tn = 0;
float fn = 0;
float precision = 0;
float fpr = 0;
float err = 0;

float sum_precision = 0;
float average_precision = 0;
float sum_fpr = 0;
float average_fpr = 0;
float sum_err = 0;
float average_err = 0;

LinkedList PolarGrid[num_bins_per_section][num_sections];

int i = 0;

float frame_iterator = 1;


pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud1(new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud2(new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);


pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;

// Passo 3 

    float s_x_s=0;
    float s_z=0;
    float s_x=0;
    float s_x_z=0;
    float n_gp=0;

    float b0=0;
    float b1=0;
    
    vector <pcl::PointXYZI> zxprojection; 
    vector <pcl::PointXYZI> zxprojection_temp;
    

void OccupancyGrid (pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud, AlfaNode * node)
{

    for (auto &point : *input_cloud){
        // Calculate distance to point


        float distance = sqrt(pow(point.x, 2) + pow(point.y, 2));
        
        float angle = atan2(point.y, point.x);
        

        if (angle < 0) {
        
            angle = angle + (2 * pi);        
        }

        if(angle >= 6.235)
            angle = 6.234;
    
        n_section = (int)(angle / delta_ang);


    
        //Number of the bin int the section
        if (distance > 1 && distance < 5) {
            m_bin = (int)distance - 1;

            PolarGrid[m_bin][n_section].insertNode(point);

        }
        //cout << "Label: " << point.Label << endl;
    }
    
    cout << "deleting list" << endl;

    for(int i = 0; i < num_bins_per_section; i++){
        for(int j = 0; j < num_sections; j++){
            if(PolarGrid[i][j].get_bin_status()==1){ 
            	zxprojection_temp = PolarGrid[i][j].outputBin(node);
                    
                zxprojection.insert(zxprojection.end(), zxprojection_temp.begin(), zxprojection_temp.end() );
            
            }
            PolarGrid[i][j].deleteList();
        }
    }
    
    
	for(std::vector<pcl::PointXYZI>::size_type i = 0; i < zxprojection.size(); i++){

		zxprojection[i].intensity = 1;
		s_x_s += zxprojection[i].intensity * pow(zxprojection[i].x,2);
		s_z += zxprojection[i].intensity * zxprojection[i].z;
		s_x += zxprojection[i].intensity * zxprojection[i].x;
		s_x_z += zxprojection[i].intensity * zxprojection[i].x * zxprojection[i].z;  
		n_gp +=1;  
    	}    

	b0 = ((s_x_s * s_z) - (s_x * s_x_z))/((n_gp*s_x_s)-pow(s_x,2));
	b1 = ((n_gp*s_x_z) - (s_x * s_z))/((n_gp*s_x_s)-pow(s_x,2));    


	cout << "b0=" << b0 << endl;
	cout << "b1=" << b1 << endl;
	cout << "###############################" << endl;

    s_x_s=0;	
    s_z=0;
    s_x=0;
    s_x_z=0;
    n_gp=0;
    zxprojection.clear();

}


void handler(AlfaNode *node)
{
    std::cout << "Point cloud received" << std::endl;
    // Simple example code aplying a distance filter to the pointcloud

    for (auto &point1 : *node->input_cloud){
        // Calculate distance to point

        float distance1 = sqrt(pow(point1.x, 2) + pow(point1.y, 2));
        
        if (distance1 > 1 && distance1 < 5) {
            
            input_cloud1->points.push_back(point1);
        }
        /*else if(point1.z < -1.8){
            node->push_point_output_cloud(point1);
        }*/

    }

    #ifdef ICP
        cout << "entering ICP" << endl;
        if(i!=0){
            icp.setMaxCorrespondenceDistance(0.05);  // Maximum distance between corresponding points
            icp.setTransformationEpsilon(1e-8);      // Minimum difference between consecutive transformations
            icp.setEuclideanFitnessEpsilon(1);       // Minimum difference between consecutive fitness scores
            icp.setMaximumIterations(5); 
            icp.setInputSource(input_cloud2);
            icp.setInputTarget(input_cloud1);

            cout << "before alignment" << endl;
            icp.align(*output_cloud);

            if (icp.hasConverged())
            {
                cout << "ICP converged \n";
            }
            else
            {
                cout << "ICP did not converge!\n";
            }
        
        }else
            *output_cloud = *input_cloud1;
    #endif
    #ifndef ICP
        *output_cloud = *input_cloud1;
    #endif
    
   OccupancyGrid(output_cloud, node);
     
    *input_cloud2 = *input_cloud1;
    input_cloud1->clear();
    output_cloud->clear();
    i++;
}

void post_processing (AlfaNode * node)
{
    node->publish_output_cloud();

    alfa_msg::msg::AlfaMetrics output_metrics;

    output_metrics.metrics.push_back(node->get_handler_time());
    output_metrics.metrics.push_back(node->get_full_processing_time());

    node->publish_metrics(output_metrics);
}




int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    //Prepare the distance filter default configuration
    AlfaExtensionParameter parameters [10];

    parameters[0].parameter_value = 5;
    parameters[0].parameter_name = "min_distance";
    parameters[1].parameter_value = 20;
    parameters[1].parameter_name = "max_distance";

    //Launch Dummy with:
    std::cout << "Starting dummy node with the following characteristics" << std::endl;
    std::cout << "Subscriber topic: /velodyne_points" << std::endl;
    std::cout << "Name of the node: dummy" << std::endl;
    std::cout << "Parameters: parameter list" << std::endl;
    std::cout << "ID: 0" << std::endl;
    std::cout << "Hardware Driver (SIU): false" << std::endl;
    std::cout << "Hardware Extension: false" << std::endl;
    std::cout << "Distance Resolution: 1 cm" << std::endl;
    std::cout << "Intensity Multiplier: 1x" << std::endl;
    std::cout << "Handler Function: handler" << std::endl;
    std::cout << "Post Processing Function: post_processing" << std::endl;
    //alfa_node dummy("/os_cloud_nodee/points","alfa_dummy", parameter, 0, false, false, 1, 1, &handler,&post_processing);

    rclcpp::spin(std::make_shared<AlfaNode>("/velodyne_points","dummy", parameters, 0, AlfaHardwareSupport{false, false}, 1, 1, &handler, &post_processing));
    rclcpp::shutdown();
    return 0;
}

