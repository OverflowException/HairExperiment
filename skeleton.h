#pragma once

#include <iostream>
#include <map>
#include <set>
#include <string>
#include <vector>

#include "LinearMath/btTransform.h"

struct Joint {
    std::string      name;
    int              id;
    int              parent;
    std::vector<int> children;
    
    btVector3        t_local;
    btQuaternion     r_local;
    btTransform      trans_local;

    btVector3        t_model;
    btQuaternion     r_model;
    btTransform      trans_model;
};

typedef std::map<int, Joint> Skeleton;
