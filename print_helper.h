#pragma once

#include <iostream>

#include "skeleton.h"

template<typename T>
std::ostream& operator<<(std::ostream& os, const std::vector<T>& vec) {
    os << "[";
    for (const T& ele : vec) {
        os << ele << ", ";
    }
    os << "]";
    return os;
}

std::ostream& operator<<(std::ostream& os, const btVector3& vec) {
    os << "[" << vec.x() << ", " << vec.y() << ", " << vec.z() << "]";
    return os;
}

std::ostream& operator<<(std::ostream& os, const btQuaternion& quat) {
    os << "[axis = " << quat.getAxis() << ", angle = " << quat.getAngle()
       << ", degree = " << quat.getAngle()  / 180 * SIMD_PI << "]";
    return os;
}

std::ostream& operator<<(std::ostream& os, const btTransform& trans) {
    os << "[translation = " << trans.getOrigin()
       << ", rotation = " << trans.getRotation() << "]";
    return os;
}

std::ostream& operator<<(std::ostream& os, const Joint& j) {
    os << "\tid = " << j.id << "\n"
       << "\tparent = " << j.parent << "\n"
       << "\tchildren = " << j.children << "\n"
       << "\tt_local = " << j.t_local << "\n"
       << "\tr_local = " << j.r_local << "\n"
       << "\ttrans_local = " << j.trans_local << "\n"
       << "\tt_model = " << j.t_model << "\n"
       << "\tr_model = " << j.r_model << "\n"
       << "\ttrans_model = " << j.trans_model;
    return os;
}
