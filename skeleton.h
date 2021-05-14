#pragma once

#include <map>

#include "LinearMath/btTransform.h"

struct Joint {
    int              id;
    int              parent;
    std::vector<int> children;
    btTransform      trans_ref_root;
};

typedef std::map<int, Joint> Skeleton;
