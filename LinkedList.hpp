
#ifndef LinkedList_hpp
#define LinkedList_hpp

#include "Node.hpp"
#include <vector>

class LinkedList{
    
    Node* head;
    
    // Variáveis para a a weighted sum of the partial evidences in terms of the sigmoid function
    int num_points;
    float height_sum;
    float average_height;
    float average_height_temp;
    int sum_average_heights;
    float height_deviation;
    float deviation;
    float weighted_sum_partial_evidences;
    float term1;
    float term2;
    float term3;
    
    float dif_average_and_actual_height;
  
    //float temp[300];

    // Variaveis para a probabilidade de ser ground, unknown or non-ground

    double all_p_values; //Corresponde a todos L_ji
    double actual_p_value; //Corresponde ao L_ji atual
    double p_state;
    int status;
    
    // Passo3 -> Weighted Linear Regression
    vector <pcl::PointXYZI> zy_system;


public:
    LinkedList();
    void insertNode(pcl::PointXYZI);
    void printList();
    int get_bin_status();
    void deleteList();
    vector <pcl::PointXYZI> outputBin(AlfaNode * node);
    float get_average_height();
    
};

#endif /* LinkedList_hpp */
