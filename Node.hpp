//
//  Node.hpp
//  linked_list
//
//  Created by Fábio Oliveira on 08/10/2022.
//

#ifndef Node_hpp
#define Node_hpp

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "alfa_node/alfa_node.hpp"
#include <stdio.h>
#include <iostream>
using namespace std;

/*typedef struct {
    float x;
    float y;
    float z;
}Point;*/

class Node{
    
    public:

        pcl::PointXYZI Point;
        Node* next;
    
        Node();
        Node(pcl::PointXYZI Point);

};

#endif /* Node_hpp */
