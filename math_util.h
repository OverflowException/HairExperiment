#pragma once

#include "LinearMath/btQuaternion.h"
#include "LinearMath/btVector3.h"

class MathUtil {
public:
    inline static btScalar deg2rad(btScalar deg) { return deg * SIMD_PI / 180.0; }

    static btQuaternion rot_between(const btVector3& src, const btVector3& dest);
    
    static btVector3 find_head_axis(const btVector3& face, const btVector3& up,  const btScalar& tilt);
};
