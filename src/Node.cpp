//
//  Node.cpp
//  linked_list
//
//  Created by Fábio Oliveira on 08/10/2022.
//

#include "Node.hpp"

Node::Node(){
    next = NULL;
}

Node::Node(pcl::PointXYZI Point){
    this -> Point = Point;
    next = NULL;
}



