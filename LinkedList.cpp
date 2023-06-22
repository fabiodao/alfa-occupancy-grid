
#include "LinkedList.hpp"
#include<algorithm>
#include<iostream>

using namespace std;

LinkedList::LinkedList(){
    num_points = 0;
    height_sum = 0;
    average_height = 0;
    average_height_temp = 0;
    sum_average_heights = 0;
    height_deviation = 0;
    deviation = 0;
    weighted_sum_partial_evidences = 0;
    dif_average_and_actual_height = 0;
    all_p_values = 0; //Corresponde a todos L_ji
    actual_p_value = 0; //Corresponde ao L_ji atual
    p_state = 0;
    status = 0;
    term1 = 0;
    term2 = 0;
    term3 = 0;
    head=NULL;
 
    }

void LinkedList::insertNode(pcl::PointXYZI point){
    
    Node* newNode = new Node(point);
    
    if (head == NULL) {
        head = newNode;
       }
    
    else {
        // Traverse till end of list
        Node* temp = head;
        while (temp->next != NULL) {

            // Update temp
            temp = temp->next;
        }

        // Insert at the last.
        temp->next = newNode;
    }

    // Height Average 
    num_points += 1;
    height_sum += point.z;
    average_height = height_sum / num_points;
    
    
  
}

void LinkedList::printList()
{
    Node* temp = head;
  
    // Check for empty list.
    if (head == NULL) {
        cout << "List empty" << endl;
        return;
    }
  
    /*// Traverse the list.
    while (temp != NULL) {
        cout << "  ----X: " << temp->point.x << "  ----Y: " << temp->point.y << "  ----Z:" << temp->point.z << " " << endl;;
        temp = temp->next;
    }*/
}

int LinkedList::get_bin_status() {
    
    deviation = 0;
    Node* temp = head;
  
    // Check for empty list.
    if (head == NULL) {
        return -1;
    }
  
    // Traverse the list.
    while (temp != NULL) {
        deviation += pow((temp->Point.z - average_height),2);;
        temp = temp->next;
    }
    
    
    height_deviation = sqrt(deviation / num_points);

    //cout << "height deviation: " << height_deviation << endl;
         
    // Logs odds values of being ground, unknown or non-ground
    
    term1 =  0.3 / (1 + exp(-6 * (average_height + 1.7)));
    term2 = (0.3 / (1 + exp(-6 * (height_deviation - 0.1))));
    term3 = (0.3 * (1-(1/(1 + exp(-4 * (num_points - 200))))));
    
    weighted_sum_partial_evidences = term1 + term2 + term3 ;
    
    //cout << "prob: " << weighted_sum_partial_evidences << endl;
    //cout << "Term1: " << term1 << endl;
    //cout << "Term2: " << term2 << endl;    
    //cout << "Term3: " << term3 << endl;
    
    if (weighted_sum_partial_evidences > 0 && weighted_sum_partial_evidences <= 0.2)
        actual_p_value = -1.39;
    else if (weighted_sum_partial_evidences > 0.2 && weighted_sum_partial_evidences < 0.8)
        actual_p_value = 0;
    else if (weighted_sum_partial_evidences >= 0.8 && weighted_sum_partial_evidences < 1)
        actual_p_value = 1.39;




    // Clamping update policy
    all_p_values = max(min(actual_p_value + all_p_values, 3.89), -3.89);

    // Status of the bin based on the probability
    p_state = exp(all_p_values) / (1 + exp(all_p_values));


    //cout << "state: " << p_state << endl;

  /*  if (p_state > 0 && p_state <= 0.3)
        status = 1;     // The bin is ground
    if (p_state > 0.3 && p_state < 0.7)
        status = 0;     // The bin is unknown
    if (p_state >= 0.7 && p_state < 1)
        status = -1;    // The bin is non-ground
*/

    if (height_deviation < 0.07 && average_height < -1.6)
        status = 1;     // The bin is ground
    else
        status = 0;     // The bin is unknown



    return status;
}

void LinkedList::deleteList() {

    Node* temp = head;
    Node* nextNode;

    while (temp != NULL) {
        nextNode = temp->next;
                
        delete temp;
        temp = nextNode;
        
    }

    head = nullptr;
    num_points = 0;
    num_points = 0;
    height_sum = 0;
    average_height = 0;
    height_deviation = 0;
    deviation = 0;
    weighted_sum_partial_evidences = 0;
    dif_average_and_actual_height = 0;
    actual_p_value = 0; //Corresponde ao L_ji atual
    p_state = 0;
    status = 0;
    zy_system.clear();
}

vector <pcl::PointXYZI> LinkedList::outputBin(AlfaNode * node){

    Node* temp = head;
  
    // Check for empty list.
    if (head == NULL) {
        return zy_system;
    }
  
    // Traverse the list.
    while (temp != NULL) {
    	

  	node->push_point_output_cloud(temp->Point);
  	
       	zy_system.push_back(temp->Point);
        
        temp = temp->next;
    }
    return zy_system;
}

float LinkedList::get_average_height(){
	return average_height;
}
